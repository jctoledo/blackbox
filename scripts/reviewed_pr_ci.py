#!/usr/bin/env python3
"""Trusted dispatch guard and commit-status publisher for reviewed PR CI."""
import json
import os
import re
import sys
from urllib.request import Request, urlopen

CONTEXT = 'lat/reviewed-ci'


def validate_inputs(number, sha):
    """Accept only a positive PR number and an immutable Git object name."""
    if not re.fullmatch(r'[1-9][0-9]*', number):
        raise ValueError('PR number must be a positive integer')
    if not re.fullmatch(r'[0-9a-f]{40}', sha):
        raise ValueError('Reviewed SHA must be a full lowercase 40-character SHA')


def validate_pr(pr, repository, sha):
    """Refuse anything other than the current open, same-repo main PR head."""
    if (pr.get('state') != 'open'
            or (pr.get('base', {}).get('repo') or {}).get('full_name') != repository
            or pr.get('base', {}).get('ref') != 'main'
            or (pr.get('head', {}).get('repo') or {}).get('full_name') != repository
            or pr.get('head', {}).get('sha') != sha):
        raise ValueError('PR must be open, target main in this repository, and match the reviewed head SHA')


def status_for_result(result):
    """Only the complete reusable validation job may report success."""
    return {'success': 'success', 'failure': 'failure'}.get(result, 'error')


class GitHub:
    def __init__(self):
        self.root = os.environ.get('GITHUB_API_URL', 'https://api.github.com').rstrip('/')
        self.repository = os.environ['GITHUB_REPOSITORY']

    def request(self, path, payload=None):
        request = Request(
            f'{self.root}/repos/{self.repository}/{path}',
            data=None if payload is None else json.dumps(payload).encode(),
            headers={'Authorization': 'Bearer ' + os.environ['GH_TOKEN'],
                     'Accept': 'application/vnd.github+json',
                     'X-GitHub-Api-Version': '2022-11-28',
                     'Content-Type': 'application/json'})
        with urlopen(request, timeout=30) as response:
            return json.load(response)


def guard(api, number, sha):
    """Check input before API access; never fall back after an API failure."""
    validate_inputs(number, sha)
    validate_pr(api.request(f'pulls/{number}'), api.repository, sha)


def publish(api, sha, state, description):
    url = (f"{os.environ['GITHUB_SERVER_URL']}/{api.repository}/actions/runs/"
           f"{os.environ['GITHUB_RUN_ID']}/attempts/{os.environ['GITHUB_RUN_ATTEMPT']}")
    api.request(f'statuses/{sha}', {'state': state, 'context': CONTEXT,
                                 'description': description, 'target_url': url})


def main():
    action = sys.argv[1]
    number, sha = os.environ['PR_NUMBER'], os.environ['REVIEWED_SHA']
    api = GitHub()
    if action in ('guard', 'pending'):
        guard(api, number, sha)
        if action == 'pending':
            publish(api, sha, 'pending', 'Reviewed commit queued for full CI on lat')
            with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
                output.write(f'sha={sha}\n')
    elif action == 'finish':
        validate_inputs(number, sha)
        state = status_for_result(os.environ['VALIDATION_RESULT'])
        # A moved/closed PR cannot be reported as a current passing review.
        try:
            guard(api, number, sha)
        except ValueError:
            state = 'error'
        publish(api, sha, state, f'Reviewed CI: {state} (validation: {os.environ["VALIDATION_RESULT"]})')
    else:
        raise ValueError('Unknown controller action')


if __name__ == '__main__':
    main()
