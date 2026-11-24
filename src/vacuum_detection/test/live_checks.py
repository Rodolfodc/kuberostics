"""The checks shared by the two live-graph launch tests (not a test module)."""
import launch_support as ls


class LiveChecks:
    """Mixin (not a TestCase, so it is not collected by itself): subclasses are `(LiveChecks, unittest.TestCase)` and
    set OFFLOAD; the graph itself is started by generate_test_description of their module."""
    OFFLOAD = None
    probe = None

    @classmethod
    def setUpClass(cls):
        cls.probe = ls.Probe()
        cls.active = cls.probe.wait_for(cls.probe.manager_active, ls.STARTUP_TIMEOUT_S, 0.5)

    @classmethod
    def tearDownClass(cls):
        cls.probe.close()

    def test_01_manager_reports_active(self):
        """009a-R01, D-26: lifecycle_manager_optional configured and activated every node of the chain."""
        self.assertTrue(self.active, 'lifecycle_manager_optional never reported is_active == true')

    def test_02_only_the_selector_publishes_the_final_topic(self):
        """009a-R01 and the contract (max_publishers 1, owner detection_selector), with check_topic."""
        self.assertTrue(self.probe.wait_for(
            lambda: len(self.probe.node.get_publishers_info_by_topic(ls.FINAL_TOPIC)) >= 1, 30.0))
        self.assertEqual(self.probe.contract_problems(ls.FINAL_TOPIC), [])
        names = [p.node_name for p in self.probe.node.get_publishers_info_by_topic(ls.FINAL_TOPIC)]
        self.assertEqual(names, ['detection_selector'])

    def test_03_the_local_detector_publishes_the_local_topic(self):
        """009a-R04: the hot-standby detector is in the graph in both modes, on the contract topic."""
        self.assertTrue(self.probe.wait_for(
            lambda: len(self.probe.node.get_publishers_info_by_topic(ls.LOCAL_TOPIC)) >= 1, 30.0))
        self.assertEqual(self.probe.contract_problems(ls.LOCAL_TOPIC), [])
        names = [p.node_name for p in self.probe.node.get_publishers_info_by_topic(ls.LOCAL_TOPIC)]
        self.assertEqual(names, ['detector_local'])

    def test_04_images_reach_the_final_topic_through_the_selector(self):
        """009a-R01/R02: frames on /camera come out of the selector as (empty) detections with the image stamp.
        The cloud never answers, so the breaker opens after open_threshold_s and local is forwarded."""
        self.assertTrue(self.probe.wait_for(
            lambda: self.probe.node.count_subscribers('/camera') >= 1, 30.0), 'nobody subscribes to /camera')
        self.assertTrue(self.probe.stream_camera_until(lambda: self.probe.final_count() >= 1, 20.0),
                        'no detection reached /myRobot/perception/detections')
        self.assertEqual(len(self.probe.received[0].detections), 0)

    def test_05_throttle_presence_follows_offload(self):
        """009a-R03: image_throttle (the only node that sends RGB to the cloud) exists only with offload:=true."""
        def names():
            return {n for n, ns in self.probe.node.get_node_names_and_namespaces()}
        self.assertTrue(self.probe.wait_for(
            lambda: {'detector_local', 'detection_selector'} <= names(), 30.0), 'chain nodes not discovered')
        if self.OFFLOAD:
            self.assertTrue(self.probe.wait_for(lambda: 'image_throttle' in names(), 30.0))
        else:
            self.assertNotIn('image_throttle', names())
