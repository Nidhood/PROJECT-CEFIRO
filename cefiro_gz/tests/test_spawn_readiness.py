#!/usr/bin/env python3
"""Tests for Gazebo readiness and launch sequencing; no ROS install required."""

import importlib.util
from pathlib import Path
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'wait_for_gazebo_ready.py'
SPEC = importlib.util.spec_from_file_location('wait_for_gazebo_ready', SCRIPT)
READY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READY)


class ReadyServiceTests(unittest.TestCase):
    def test_parse_services(self):
        self.assertEqual(READY.parse_services(' /world/main_world/create\n\n/foo\n'),
                         {'/world/main_world/create', '/foo'})

    def test_gate_opens_when_create_service_exists(self):
        outputs = ['', '/world/main_world/scene/info\n',
                   '/world/main_world/create\n']
        responses = [types.SimpleNamespace(returncode=0, stdout=x) for x in outputs]
        with patch.object(READY.subprocess, 'run', side_effect=responses) as run, \
                patch.object(READY.time, 'sleep'):
            self.assertEqual(READY.wait_for_create_service('main_world'), 0)
            self.assertEqual(run.call_count, 3)

    def test_different_world_does_not_unblock(self):
        responses = [
            types.SimpleNamespace(returncode=0, stdout='/world/other_world/create'),
            types.SimpleNamespace(returncode=0, stdout='/world/main_world/create'),
        ]
        with patch.object(READY.subprocess, 'run', side_effect=responses) as run, \
                patch.object(READY.time, 'sleep'):
            self.assertEqual(READY.wait_for_create_service('main_world'), 0)
            self.assertEqual(run.call_count, 2)

    def test_sequential_launch_uses_events_not_scene_polling(self):
        launch = (ROOT / 'launch' / 'spawn_models.launch.py').read_text()
        self.assertIn('target_action=spawn_desk', launch)
        self.assertIn('target_action=spawn_lamp', launch)
        self.assertIn('spawn_lamp,', launch)
        self.assertIn('spawn_robot,', launch)
        self.assertIn('event.returncode != 0', launch)
        self.assertNotIn('wait_for_scene_models', launch)
        self.assertNotIn('TimerAction', launch)
        self.assertNotIn('/scene/info', launch)
        self.assertIn("'models', 'desk', 'sdf', 'desk.sdf'", launch)
        self.assertIn("'models', 'lamp', 'sdf', 'lamp.sdf'", launch)


if __name__ == '__main__':
    unittest.main()
