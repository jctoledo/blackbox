"""Exercise the GitHub data contract without contacting GitHub or running builds."""
import copy
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'reviewed_pr_ci', Path(__file__).resolve().parents[1] / 'scripts/reviewed_pr_ci.py')
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)
SHA = 'a' * 40
REPO = 'jctoledo/blackbox'
PR = {'state': 'open', 'base': {'ref': 'main', 'repo': {'full_name': REPO}},
      'head': {'sha': SHA, 'repo': {'full_name': REPO}}}


class FakeAPI:
    repository = REPO

    def __init__(self, pr=None, error=None):
        self.pr = copy.deepcopy(PR if pr is None else pr)
        self.error = error
        self.calls = []

    def request(self, path, payload=None):
        self.calls.append((path, payload))
        if self.error:
            raise self.error
        return self.pr


class ReviewedPRTests(unittest.TestCase):
    def test_accepts_current_same_repository_head(self):
        api = FakeAPI()
        controller.guard(api, '139', SHA)
        self.assertEqual(api.calls, [('pulls/139', None)])

    def test_refuses_stale_fork_closed_wrong_base_and_repository(self):
        for field, value in [('stale', 'b' * 40), ('fork', 'other/blackbox'),
                             ('closed', 'closed'), ('base', 'release'),
                             ('repository', 'other/blackbox'), ('deleted', None)]:
            with self.subTest(field=field):
                pr = copy.deepcopy(PR)
                if field == 'stale':
                    pr['head']['sha'] = value
                elif field == 'fork':
                    pr['head']['repo']['full_name'] = value
                elif field == 'deleted':
                    pr['head']['repo'] = value
                elif field == 'closed':
                    pr['state'] = value
                elif field == 'base':
                    pr['base']['ref'] = value
                else:
                    pr['base']['repo']['full_name'] = value
                api = FakeAPI(pr)
                with self.assertRaises(ValueError):
                    controller.guard(api, '139', SHA)
                self.assertEqual(len(api.calls), 1)

    def test_malformed_input_refused_before_api(self):
        for number, sha in [('0', SHA), ('1;echo', SHA), ('01', SHA),
                            ('1', 'abc'), ('1', 'A' * 40), ('1', SHA + '\n')]:
            api = FakeAPI()
            with self.subTest(number=number, sha=sha), self.assertRaises(ValueError):
                controller.guard(api, number, sha)
            self.assertEqual(api.calls, [])

    def test_api_failure_does_not_authorize(self):
        with self.assertRaises(TimeoutError):
            controller.guard(FakeAPI(error=TimeoutError()), '139', SHA)

    def run_action(self, api, action, result='success'):
        with tempfile.NamedTemporaryFile() as output:
            env = {'PR_NUMBER': '139', 'REVIEWED_SHA': SHA,
                   'GITHUB_OUTPUT': output.name, 'VALIDATION_RESULT': result,
                   'GITHUB_SERVER_URL': 'https://github.com', 'GITHUB_RUN_ID': '42',
                   'GITHUB_RUN_ATTEMPT': '2'}
            with patch.dict(os.environ, env), patch.object(controller, 'GitHub', return_value=api), \
                    patch('sys.argv', ['reviewed_pr_ci.py', action]):
                controller.main()
            return Path(output.name).read_text()

    def test_pending_publishes_exact_sha_and_attempt_url_before_output(self):
        api = FakeAPI()
        self.assertEqual(self.run_action(api, 'pending'), f'sha={SHA}\n')
        path, status = api.calls[-1]
        self.assertEqual(path, f'statuses/{SHA}')
        self.assertEqual(status['state'], 'pending')
        self.assertEqual(status['context'], 'lat/reviewed-ci')
        self.assertEqual(status['target_url'],
                         f'https://github.com/{REPO}/actions/runs/42/attempts/2')

    def test_refusal_and_api_failure_publish_no_pending(self):
        pr = copy.deepcopy(PR)
        pr['head']['sha'] = 'b' * 40
        for api, error in [(FakeAPI(pr), ValueError),
                           (FakeAPI(error=TimeoutError()), TimeoutError)]:
            with self.assertRaises(error):
                self.run_action(api, 'pending')
            self.assertTrue(all(payload is None for _, payload in api.calls))

    def test_final_mapping_never_treats_skipped_cancelled_or_timeout_as_success(self):
        for result, state in [('success', 'success'), ('failure', 'failure'),
                              ('cancelled', 'error'), ('skipped', 'error'),
                              ('timed_out', 'error'), ('unknown', 'error')]:
            with self.subTest(result=result):
                api = FakeAPI()
                self.run_action(api, 'finish', result)
                self.assertEqual(api.calls[-1][0], f'statuses/{SHA}')
                self.assertEqual(api.calls[-1][1]['state'], state)

    def test_head_moving_during_validation_cannot_receive_success(self):
        pr = copy.deepcopy(PR)
        pr['head']['sha'] = 'b' * 40
        api = FakeAPI(pr)
        self.run_action(api, 'finish')
        self.assertEqual(api.calls[-1][0], f'statuses/{SHA}')
        self.assertEqual(api.calls[-1][1]['state'], 'error')

    def test_final_api_failure_cannot_publish_success(self):
        api = FakeAPI(error=TimeoutError())
        with self.assertRaises(TimeoutError):
            self.run_action(api, 'finish')
        self.assertEqual(api.calls, [('pulls/139', None)])


if __name__ == '__main__':
    unittest.main()
