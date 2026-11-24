"""Pages mutation only follows exact-artifact validation; credentials stay private."""
import copy
import importlib.util
from pathlib import Path
from urllib.error import HTTPError
import unittest

SPEC = importlib.util.spec_from_file_location('deploy_pages', Path(__file__).parents[1] / 'scripts/deploy_pages.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SHA = 'a' * 40


class DeployPagesTests(unittest.TestCase):
    def fixture(self, mutate=None, statuses=None):
        artifact = dict(id=123, name='github-pages', expired=False, size_in_bytes=100,
                        workflow_run=dict(id=456, head_sha=SHA))
        if mutate:
            mutate(artifact)
        calls = []
        statuses = iter(statuses or ['deployment_in_progress', 'succeed'])

        def api(method, endpoint, payload):
            calls.append((method, endpoint, copy.deepcopy(payload)))
            if endpoint == 'actions/runs/456/artifacts?per_page=100':
                return dict(artifacts=[dict(id=122, name='github-pages'), artifact])
            if method == 'POST':
                return dict(id='deployment-1', page_url='https://example.github.io/project/')
            return dict(status=next(statuses))

        def oidc():
            calls.append(('OIDC', None, None))
            return 'private-oidc-token'

        return api, oidc, calls

    def test_exact_artifact_and_oidc_are_required_before_post_and_success(self):
        api, oidc, calls = self.fixture()
        url = MODULE.deploy(api, oidc, 123, 456, SHA, sleep=lambda _: None)
        self.assertEqual(url, 'https://example.github.io/project/')
        self.assertEqual([c[0] for c in calls], ['GET', 'OIDC', 'POST', 'GET', 'GET'])
        self.assertEqual(calls[2][2], dict(artifact_id=123, pages_build_version=SHA,
                                         oidc_token='private-oidc-token'))

    def test_stale_wrong_or_expired_artifacts_never_request_oidc_or_deploy(self):
        mutations = [lambda a: a.update(id=999), lambda a: a.update(name='other'),
                     lambda a: a.update(expired=True), lambda a: a.update(size_in_bytes=0),
                     lambda a: a['workflow_run'].update(id=999),
                     lambda a: a['workflow_run'].update(head_sha='b' * 40)]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                api, oidc, calls = self.fixture(mutate)
                with self.assertRaises(ValueError):
                    MODULE.deploy(api, oidc, 123, 456, SHA)
                self.assertEqual([c[0] for c in calls], ['GET'])

    def test_failed_and_unfinished_deployments_never_report_success(self):
        api, oidc, calls = self.fixture(statuses=['deployment_content_failed'])
        with self.assertRaises(RuntimeError):
            MODULE.deploy(api, oidc, 123, 456, SHA)
        self.assertEqual(calls[-1][:2], ('POST', 'pages/deployments/deployment-1/cancel'))
        api, oidc, calls = self.fixture(statuses=['deployment_in_progress'] * 2)
        with self.assertRaises(TimeoutError):
            MODULE.deploy(api, oidc, 123, 456, SHA, sleep=lambda _: None, attempts=2)
        self.assertEqual(calls[-1][:2], ('POST', 'pages/deployments/deployment-1/cancel'))

    def test_polling_exception_cancels_without_retrying_creation(self):
        original, oidc, calls = self.fixture()

        def api(method, endpoint, payload):
            if method == 'GET' and endpoint.startswith('pages/deployments/'):
                raise ConnectionError('temporary service failure')
            return original(method, endpoint, payload)

        with self.assertRaises(ConnectionError):
            MODULE.deploy(api, oidc, 123, 456, SHA)
        self.assertEqual(calls[-1][:2], ('POST', 'pages/deployments/deployment-1/cancel'))
        self.assertEqual(sum(c[0] == 'POST' and c[1] == 'pages/deployments' for c in calls), 1)

    def test_missing_oidc_cannot_create_deployment(self):
        api, _, calls = self.fixture()
        with self.assertRaises(ValueError):
            MODULE.deploy(api, lambda: '', 123, 456, SHA)
        self.assertEqual([c[0] for c in calls], ['GET'])

    def test_transient_status_lookup_retries_get_but_never_creation(self):
        original, oidc, calls = self.fixture(statuses=['succeed'])
        first = True

        def api(method, endpoint, payload):
            nonlocal first
            if method == 'GET' and endpoint.startswith('pages/deployments/') and first:
                first = False
                raise HTTPError('https://api.github.com', 404, 'not indexed yet', {}, None)
            return original(method, endpoint, payload)

        self.assertTrue(MODULE.deploy(api, oidc, 123, 456, SHA, sleep=lambda _: None).startswith('https:'))
        self.assertEqual(sum(c[0] == 'POST' and c[1] == 'pages/deployments' for c in calls), 1)
        self.assertFalse(any(c[1] and c[1].endswith('/cancel') for c in calls))


if __name__ == '__main__':
    unittest.main()
