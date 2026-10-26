"""Sensor boundary, time stamps and reset semantics without Genesis initialization."""
import unittest
from xrobocon.sensors import SensorSuite,SensorSample,validate_sensor_config

class SensorsTests(unittest.TestCase):
    def suite(self):
        calls=[]
        def read(robot,kind,options):calls.append((robot,kind));return {'value':[len(calls)]}
        return SensorSuite({'blue':{'lidar':{'hz':10},'imu':{'hz':100}},'red':{'imu':{'enabled':False}}},read),calls
    def test_rates_and_measurement_timestamp_survive_repeated_reads(self):
        suite,calls=self.suite();suite.update(0)
        first=suite.read('blue');self.assertEqual(first['lidar'].timestamp,0)
        suite.update(.01);again=suite.read('blue')
        self.assertEqual(again['lidar'].sequence,0);self.assertEqual(again['imu'].sequence,1)
        suite.update(.1);self.assertEqual(suite.read('blue')['lidar'].timestamp,.1)
        self.assertEqual(suite.read('red'),{})
    def test_copies_isolate_agent_mutation(self):
        suite,_=self.suite();suite.update(0);sample=suite.read('blue')
        sample['imu'].data['value'][0]=999
        self.assertNotEqual(suite.read('blue')['imu'].data['value'][0],999)
    def test_reset_one_participant_invalidates_cache_and_resets_sequence(self):
        suite,_=self.suite();suite.update(0);suite.update(.1)
        suite.reset('blue',not_before=.11)
        self.assertEqual(suite.read('blue'),{});suite.update(.1);self.assertEqual(suite.read('blue'),{})
        suite.update(.11);self.assertEqual(suite.read('blue')['imu'].sequence,0)
        self.assertEqual(suite.read('blue')['imu'].timestamp,.11)
    def test_validation_rejects_bad_rate_type_sensor_and_excessive_rate(self):
        for config in ({'x':{'unknown':{}}},{'x':{'imu':{'hz':0}}},{'x':{'imu':{'hz':101}}},{'x':{'imu':{'enabled':'yes'}}},{'x':{'lidar':{'rays':0}}},{'x':{'lidar':{'min_range':3,'max_range':2}}}):
            with self.subTest(config=config),self.assertRaises(ValueError):validate_sensor_config(config,['x'])
    def test_unknown_participant_rejected(self):
        with self.assertRaises(ValueError):validate_sensor_config({'other':{}},['blue'])
    def test_round_reset_discards_previous_clock(self):
        suite,_=self.suite();suite.update(100);suite.reset();suite.update(0)
        self.assertEqual(suite.read('blue')['imu'].timestamp,0)
        self.assertEqual(suite.read('blue')['imu'].sequence,0)
    def test_disabled_sensors_do_not_read_backend(self):
        calls=[]
        suite=SensorSuite({'blue':{'imu':{'enabled':False}}},lambda *args:calls.append(args))
        suite.update(0);self.assertEqual(calls,[]);self.assertEqual(suite.read('blue'),{})
    def test_noninteger_rate_is_quantized_without_future_timestamps(self):
        suite=SensorSuite({'blue':{'imu':{'hz':60}}},lambda *args:{})
        stamps=[]
        for tick in range(101):
            now=tick*.01;suite.update(now);sample=suite.read('blue')['imu']
            self.assertLessEqual(sample.timestamp,now)
            if not stamps or stamps[-1]!=sample.timestamp:stamps.append(sample.timestamp)
        self.assertEqual(len(stamps),61)
        self.assertLessEqual(max(b-a for a,b in zip(stamps,stamps[1:])),.020000001)
