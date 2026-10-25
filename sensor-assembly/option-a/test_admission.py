import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from admit_option_a_once import NAME, JOB_NAME, admit


class FakeSlurm:
    def __init__(self,code):
        self.code=code;self.job=None;self.other='';self.history=''
        self.calls=[];self.owner=os.getuid();self.command=str(code/'option_a_on_rosie.sh')
        self.timeout=False;self.bad_response=False;self.terminal_state='FAILED'
    def __call__(self,args):
        self.calls.append(args)
        if args[0]=='squeue':
            value=self.other or ((self.job+'|'+JOB_NAME+'|RUNNING') if self.job else '')
            return value if args[-1]=='%i|%j|%T' else '\n'.join('|'.join(v.split('|')[:2]) for v in value.splitlines())
        if args[0]=='sacct':return self.history
        if args[0]=='scontrol':
            if args[-1]=='123':return f'JobId=123 JobName=some-other-work JobState={self.terminal_state} UserId=test({self.owner})'
            return f'JobId={args[-1]} JobName={JOB_NAME} Command={self.command} UserId=test({self.owner})'
        if args[0]=='sbatch':
            if self.timeout:raise subprocess.TimeoutExpired(args,45)
            if self.bad_response:return 'unconfirmed'
            self.job='299001';return self.job
        raise AssertionError(args)
    def submits(self):return sum(x[0]=='sbatch' for x in self.calls)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.home=Path(self.temp.name).resolve()
        self.code=self.home/'team-v-tests'/NAME;self.code.mkdir(parents=True)
        (self.code/'option_a_on_rosie.sh').write_text('reviewed fixture\n')
        hashes={'option_a_on_rosie.sh':hashlib.sha256((self.code/'option_a_on_rosie.sh').read_bytes()).hexdigest()}
        (self.code/'snapshot-sha256.json').write_text(json.dumps(hashes))
        self.digest=hashlib.sha256((self.code/'snapshot-sha256.json').read_bytes()).hexdigest()
        self.fake=FakeSlurm(self.code)
    def tearDown(self):self.temp.cleanup()
    def call(self):return admit(self.code,self.digest,self.home,self.fake)
    def receipt(self):return self.home/'team-v-tests/submission-receipts'/(NAME+'.json')
    def test_once_then_existing_receipt(self):
        a=self.call();b=self.call()
        self.assertEqual(a['gpu'],b['gpu']);self.assertEqual(self.fake.submits(),1)
        self.assertEqual(a['allocation_cap_seconds'],900);self.assertEqual(a['automatic_retries'],0)
    def test_queued_other_job_rejects_without_intent(self):
        self.fake.other='123|some-other-work|PENDING'
        with self.assertRaisesRegex(RuntimeError,'Other jobs'):self.call()
        self.assertEqual(self.fake.submits(),0)
        self.assertFalse(self.receipt().exists())
    def test_uncertain_submission_not_blindly_repeated(self):
        self.fake.timeout=True
        with self.assertRaises(subprocess.TimeoutExpired):self.call()
        self.fake.timeout=False
        with self.assertRaisesRegex(RuntimeError,'Unresolved previous intent'):self.call()
        self.assertEqual(self.fake.submits(),1)
        self.assertIn('submission_intent',json.loads(self.receipt().read_text()))
    def test_accounting_recovers_accepted_but_lost_response(self):
        self.fake.history='299007|'
        a=self.call();self.assertEqual(a['gpu'],'299007');self.assertEqual(self.fake.submits(),0)
        self.assertTrue(a['recovered_from_slurm'])
    def test_wrong_owner_rejects_recovery(self):
        self.fake.history='299007|';self.fake.owner=os.getuid()+1
        with self.assertRaisesRegex(RuntimeError,'owner mismatch'):self.call()
        self.assertEqual(self.fake.submits(),0)
    def test_wrong_source_rejects_recovery(self):
        self.fake.history='299007|';self.fake.command='/another/source.sh'
        with self.assertRaisesRegex(RuntimeError,'source/owner mismatch'):self.call()
    def test_ambiguous_history_rejects(self):
        self.fake.history='299007|\n299008|'
        with self.assertRaisesRegex(RuntimeError,'Ambiguous'):self.call()
    def test_changed_or_extra_source_rejects(self):
        (self.code/'extra').write_text('unreviewed')
        with self.assertRaisesRegex(RuntimeError,'extra source'):self.call()
        (self.code/'extra').unlink();(self.code/'option_a_on_rosie.sh').write_text('changed')
        with self.assertRaisesRegex(RuntimeError,'Source mismatch'):self.call()
    def test_symlink_rejects(self):
        (self.code/'unexpected').symlink_to(self.code/'option_a_on_rosie.sh')
        with self.assertRaisesRegex(RuntimeError,'Symlink'):self.call()
    def test_bad_sbatch_response_consumes_intent(self):
        self.fake.bad_response=True
        with self.assertRaisesRegex(RuntimeError,'Uncertain sbatch'):self.call()
        with self.assertRaisesRegex(RuntimeError,'Unresolved'):self.call()
        self.assertEqual(self.fake.submits(),1)
    def test_retained_confirmed_terminal_does_not_waste_idle_GPU(self):
        self.fake.other='123|some-other-work|FAILED'
        result=self.call()
        self.assertEqual(self.fake.submits(),1)
        self.assertEqual(result['queue_guard']['retained_terminal_jobs'][0]['terminal_state'],'FAILED')
    def test_terminal_state_race_rejects(self):
        self.fake.other='123|some-other-work|FAILED';self.fake.terminal_state='RUNNING'
        with self.assertRaisesRegex(RuntimeError,'unconfirmed'):self.call()
        self.assertEqual(self.fake.submits(),0)


if __name__=='__main__':unittest.main()
