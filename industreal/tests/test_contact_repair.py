import unittest
from team_v_industreal.contacts import ContactAudit


class ContactTensorTests(unittest.TestCase):
    def test_only_indexed_contacts_count_and_both_jaws_need_force(self):
        audit=ContactAudit()
        audit.ingest_tensor(['JawL','JawR','FixedAsset','Table'],[1,1,0,0],[0,1,0,0],
                            [-.0001,-.0002,float('nan')],[.2,.25,float('nan')])
        report=audit.report()
        self.assertFalse(audit.invalid)
        self.assertTrue(report['last_bilateral_grip'])
        self.assertEqual(report['tensor_contact_points'],2)
        self.assertEqual(report['max_contact_depth_m'],.0002)
        audit.ingest_tensor(['JawL','JawR'],[1,0],[0,0],[0.],[.2])
        self.assertFalse(audit.report()['last_bilateral_grip'])
        self.assertEqual(audit.report()['bilateral_grip_fraction'],.5)

    def test_bad_ranges_nonfinite_active_contact_and_deep_contact_fail(self):
        for counts,starts,distances,forces in [([2],[0],[0.],[1.]),
            ([1],[0],[float('nan')],[1.]),([1],[0],[0.],[-1.]),
            ([1],[0],[-.0003],[1.])]:
            audit=ContactAudit();audit.ingest_tensor(['JawL'],counts,starts,distances,forces)
            self.assertTrue(audit.invalid)

    def test_compound_shapes_aggregate_without_losing_jaw_identity(self):
        audit=ContactAudit()
        audit.ingest_tensor(['JawL','JawL','JawR'],[1,1,1],[0,1,2],
                            [0.,0.,0.],[.0006,.0006,.002])
        self.assertTrue(audit.report()['last_bilateral_grip'])
        self.assertAlmostEqual(audit.report()['maximum_pair_normal_force_n']['JawL'],.0012)

    def test_no_contact_is_not_grip_and_table_contact_is_not_support_pass(self):
        audit=ContactAudit()
        self.assertFalse(audit.report()['contact_monitor_valid'])
        audit.ingest_tensor(['JawL','JawR'],[0,0],[0,0],[],[])
        self.assertFalse(audit.report()['last_bilateral_grip'])
        self.assertFalse(audit.grip_contact_observed)
        audit.ingest_tensor(['Table'],[1],[0],[0.],[.19])
        self.assertTrue(audit.invalid)

    def test_inactive_negative_starts_are_not_read_or_mistaken_for_contact(self):
        audit=ContactAudit()
        audit.ingest_tensor(['JawL','JawR','FixedAsset'],[1,1,0],[0,1,-1],
                            [-.00005,-.00005,float('nan')],[2.,2.,float('nan')])
        self.assertFalse(audit.invalid)
        self.assertTrue(audit.report()['last_bilateral_grip'])
        self.assertFalse(audit.socket_contact_observed)

    def test_shaft_grasp_keeps_zero_action_and_lift_inside_original_action_box(self):
        # The circular AA recess begins14.22mm behind the tips. At20mm,
        # 5.78mm of unchanged straight blade can engage the8mm peg shaft.
        depth=.020;straight=depth-(.00697+.00725)
        self.assertGreater(straight,.005)
        initial=.050-depth+.005
        self.assertLess(initial+.005+.001+3*.001,.050)
        opening=.007986/2-.00005
        self.assertTrue(0.<opening<.006)
        self.assertAlmostEqual(40000*(.007986/2-opening),2.)

    def test_signed_contacts_require_independent_vector_corroboration(self):
        audit=ContactAudit()
        audit.ingest_tensor(['JawL','JawR'],[1,1],[0,1],[-.0001,-.0001],[-2.,2.],
                            [[0.,1.,0.],[0.,1.,0.]],[[0.,-2.,0.],[0.,2.,0.]])
        self.assertFalse(audit.invalid)
        self.assertTrue(audit.last_bilateral_grip)
        self.assertEqual(audit.force_crosscheck_frames,1)
        self.assertEqual(audit.last_pair_force_world_n['JawL'],[0.,-2.,0.])
        wrong=ContactAudit()
        wrong.ingest_tensor(['JawL'],[1],[0],[0.],[-2.],[[0.,1.,0.]],[[0.,2.,0.]])
        self.assertTrue(wrong.invalid)

    def test_zero_count_matrix_must_not_claim_force(self):
        audit=ContactAudit()
        audit.ingest_tensor(['JawL'],[0],[-1],[],[],[],[[0.,2.,0.]])
        self.assertTrue(audit.invalid)

    def test_unaccounted_support_force_is_rejected(self):
        for net,invalid in [([0.,0.,0.],False),([0.,0.,.19],True)]:
            audit=ContactAudit()
            audit.ingest_tensor(['JawL','JawR'],[1,1],[0,1],[-.0001,-.0001],[-2.,2.],
                                [[0.,1.,0.],[0.,1.,0.]],[[0.,-2.,0.],[0.,2.,0.]],net)
            self.assertEqual(audit.invalid,invalid)
            self.assertEqual(audit.net_force_coverage_frames,1)

    def test_zero_force_separated_candidates_do_not_claim_touch(self):
        audit=ContactAudit()
        audit.ingest_tensor(['FixedAsset','JawL'],[1,1],[0,1],[.004,.001],[0.,0.])
        self.assertTrue(audit.socket_contact_candidates_observed)
        self.assertFalse(audit.socket_contact_observed)
        self.assertFalse(audit.grip_contact_observed)
        audit.ingest_tensor(['FixedAsset'],[1],[0],[-.00001],[0.])
        self.assertTrue(audit.socket_contact_observed)


class ActivePrefixTests(unittest.TestCase):
    def test_prefix_preserves_last_active_point_and_inactive_garbage_is_ignored(self):
        from team_v_industreal.contacts import active_prefix_length
        self.assertEqual(active_prefix_length([2,0,3],[5,-1,10],20),13)
        self.assertEqual(active_prefix_length([0],[-1],4096),0)
        for c,st,n in [([2],[19],20),([1],[-1],20),([.5],[0],20),([1],[float('nan')],20)]:
            with self.subTest(c=c,st=st),self.assertRaises(ValueError):active_prefix_length(c,st,n)
    def test_full_and_prefix_buffers_produce_identical_force_and_depth_audit(self):
        from team_v_industreal.contacts import active_prefix_length,ContactAudit
        names=['JawL','JawR'];counts=[2,1];starts=[1,5]
        distances=[float('nan'),-.00001,-.0002,float('nan'),float('nan'),-.00002]+[float('nan')]*4090
        forces=[float('nan'),1.,2.,float('nan'),float('nan'),3.]+[float('nan')]*4090
        a=ContactAudit();a.ingest_tensor(names,counts,starts,distances,forces)
        end=active_prefix_length(counts,starts,len(forces));b=ContactAudit();b.ingest_tensor(names,counts,starts,distances[:end],forces[:end])
        self.assertEqual(a.report(),b.report());self.assertEqual(b.max_depth_m,.0002)
