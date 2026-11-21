#!/usr/bin/env python3
"""Wait for an uploaded artifact to appear in the run's REST index."""
import json
import os
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def wait_for_artifact(fetch, name, timeout=60, interval=2, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    while True:
        try:
            artifacts = fetch().get('artifacts', [])
            for artifact in artifacts:
                if artifact.get('name') == name and not artifact.get('expired', False):
                    return artifact['id']
        except HTTPError as error:
            # Authentication/permission failures are not index propagation delays.
            if error.code not in (404, 429, 500, 502, 503, 504):
                raise
        except URLError:
            pass
        remaining = deadline - clock()
        if remaining <= 0:
            raise TimeoutError(f'Artifact {name!r} was not indexed within {timeout} seconds')
        sleep(min(interval, remaining))


def main():
    name = sys.argv[1]
    api = os.environ.get('GITHUB_API_URL', 'https://api.github.com').rstrip('/')
    repository = os.environ['GITHUB_REPOSITORY']
    run = os.environ['GITHUB_RUN_ID']
    request = Request(f'{api}/repos/{repository}/actions/runs/{run}/artifacts?per_page=100',
                      headers={'Authorization': 'Bearer ' + os.environ['GH_TOKEN'],
                               'Accept': 'application/vnd.github+json'})

    def fetch():
        with urlopen(request, timeout=10) as response:
            return json.load(response)

    print(f'Waiting for uploaded artifact {name!r} to appear in the run index', flush=True)
    artifact_id = wait_for_artifact(fetch, name)
    print(f'Artifact {artifact_id} is indexed', flush=True)


if __name__ == '__main__':
    main()
