#!/usr/bin/env python3
"""Deploy the exact uploaded Pages artifact via REST, retaining OIDC authorization."""
import json
import os
import re
import signal
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError


def validate_artifact(artifact, artifact_id, run_id, sha):
    if (artifact.get('id') != artifact_id or artifact.get('name') != 'github-pages'
            or artifact.get('expired') is not False
            or not 0 < artifact.get('size_in_bytes', 0) <= 1_000_000_000
            or artifact.get('workflow_run', {}).get('id') != run_id
            or artifact.get('workflow_run', {}).get('head_sha') != sha):
        raise ValueError('Pages artifact does not match the uploaded run and source')


def deploy(api, oidc, artifact_id, run_id, sha, sleep=time.sleep, attempts=120, clock=time.monotonic):
    """Validate before requesting OIDC or mutating Pages. Never retry a POST."""
    artifact = api('GET', f'actions/artifacts/{artifact_id}', None)
    validate_artifact(artifact, artifact_id, run_id, sha)
    token = oidc()
    if not isinstance(token, str) or not token:
        raise ValueError('OIDC token unavailable')
    created = api('POST', 'pages/deployments', {
        'artifact_id': artifact_id, 'pages_build_version': sha, 'oidc_token': token,
    })
    deployment_id = str(created.get('id', ''))
    page_url = created.get('page_url', '')
    if not re.fullmatch(r'[0-9A-Za-z_-]{1,128}', deployment_id):
        raise ValueError('Invalid Pages deployment response')
    failed = {'deployment_failed', 'deployment_perms_error', 'deployment_content_failed',
              'deployment_cancelled', 'deployment_lost'}
    completed = False
    deadline = clock() + 600
    try:
        if not isinstance(page_url, str) or not re.fullmatch(r'https://[^\s]+', page_url):
            raise ValueError('Invalid Pages deployment URL')
        for attempt in range(attempts):
            if clock() >= deadline:
                break
            status = api('GET', f'pages/deployments/{deployment_id}', None).get('status')
            if status == 'succeed':
                completed = True
                return page_url
            if status in failed:
                raise RuntimeError('Pages reported a terminal deployment failure')
            if attempt + 1 < attempts:
                sleep(min(5, max(0, deadline - clock())))
        raise TimeoutError('Pages did not report success before the deployment timeout')
    finally:
        if not completed:
            # Do not leave an older deployment publishing after the concurrency slot releases.
            try:
                api('POST', f'pages/deployments/{deployment_id}/cancel', None)
            except Exception:
                pass


def request_json(url, token, method='GET', payload=None):
    data = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=data, method=method, headers={
        'Authorization': 'Bearer ' + token, 'Accept': 'application/vnd.github+json',
        'Content-Type': 'application/json', 'User-Agent': 'blackbox-pages-deployer',
    })
    with urlopen(request, timeout=30) as response:
        return json.load(response)


def main():
    def interrupted(_signum, _frame):
        raise InterruptedError('Deployment job interrupted')

    signal.signal(signal.SIGTERM, interrupted)
    env = os.environ
    artifact_id, run_id = int(env['PAGES_ARTIFACT_ID']), int(env['GITHUB_RUN_ID'])
    sha = env['GITHUB_SHA']
    if artifact_id <= 0 or run_id <= 0 or not re.fullmatch(r'[a-f0-9]{40}', sha):
        raise ValueError('Invalid deployment identity')
    base = env.get('GITHUB_API_URL', 'https://api.github.com').rstrip('/')
    repository = env['GITHUB_REPOSITORY']

    def api(method, endpoint, payload):
        return request_json(f'{base}/repos/{repository}/{endpoint}', env['GH_TOKEN'], method, payload)

    def oidc():
        return request_json(env['ACTIONS_ID_TOKEN_REQUEST_URL'],
                            env['ACTIONS_ID_TOKEN_REQUEST_TOKEN'])['value']

    page_url = deploy(api, oidc, artifact_id, run_id, sha)
    with open(env['GITHUB_OUTPUT'], 'a', encoding='utf-8') as output:
        output.write('page_url=' + page_url + '\n')
    print('Verified artifact deployed successfully to GitHub Pages')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        # Request URLs and OIDC payloads must never appear in error tracebacks.
        reason = f'HTTP {error.code}' if isinstance(error, HTTPError) else type(error).__name__
        print(f'Pages deployment failed ({reason})', file=sys.stderr)
        sys.exit(1)
