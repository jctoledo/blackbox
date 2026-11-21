import importlib.util
from pathlib import Path
import unittest
from urllib.error import HTTPError

spec = importlib.util.spec_from_file_location('artifact_wait', Path(__file__).parents[1] / 'scripts/wait_for_artifact.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ArtifactWait(unittest.TestCase):
    def test_delayed_index_and_expired_artifact(self):
        now = [0]
        replies = iter([{'artifacts': []}, {'artifacts': [{'name': 'pages', 'expired': True, 'id': 1}]},
                        {'artifacts': [{'name': 'pages', 'expired': False, 'id': 2}]}])
        result = module.wait_for_artifact(lambda: next(replies), 'pages', clock=lambda: now[0],
                                          sleep=lambda seconds: now.__setitem__(0, now[0] + seconds))
        self.assertEqual(result, 2)
        self.assertEqual(now[0], 4)

    def test_bounded_failure_without_a_matching_artifact(self):
        now = [0]
        with self.assertRaises(TimeoutError):
            module.wait_for_artifact(lambda: {'artifacts': [{'name': 'other', 'id': 1}]}, 'pages', timeout=5,
                                     clock=lambda: now[0], sleep=lambda seconds: now.__setitem__(0, now[0] + seconds))
        self.assertEqual(now[0], 5)

    def test_permission_failure_is_not_hidden(self):
        def forbidden():
            raise HTTPError('https://api.github.com/example', 403, 'Forbidden', {}, None)
        with self.assertRaises(HTTPError):
            module.wait_for_artifact(forbidden, 'pages')
