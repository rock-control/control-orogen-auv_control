require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'auv_control::PIDController' do
    include Orocos::Test::Component
    start  'pid', 'auv_control::PIDController' => 'pid'
    reader 'pid', 'cmd_out', :attr_name => 'cmd_out'
    writer 'pid', 'cmd_in', :attr_name => 'cmd_in'
    writer 'pid', 'pose_samples', :attr_name => 'pose_samples'

    def generate_default_pose
        pose_sample = pid.pose_samples.new_sample
        pose_sample.position = pose_sample.velocity = pose_sample.angular_velocity = Eigen::Vector3.new(0, 0, 0)
        pose_sample.orientation = Eigen::Quaternion.Identity
        pose_sample.time = Time.now
        pose_sample
    end

    def generate_default_cmd
        set_point = pid.cmd_in.new_sample
        set_point.linear = Eigen::Vector3.new(1, 0, 0)
        set_point.angular = Eigen::Vector3.new(0, 0, 0)
        set_point.time = Time.now
        set_point
    end

    it "should not provide output samples with same timestamp" do

        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        pid.start

        pose_sample = generate_default_pose
        set_point = generate_default_cmd

        cmd_in.write set_point

        for i in 0..3
            pose_sample.time = Time.now
            set_point.time = Time.now
            puts "pose_sample.time: #{pose_sample.time} + #{pose_sample.time.usec}"
            puts "set_point.time: #{set_point.time} + #{set_point.time.usec}"
            pose_samples.write pose_sample
            cmd_in.write set_point
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            puts "cmd_out.time: #{cmd_out0.time} + #{cmd_out0.time.usec}"
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.01
            end
            puts " "
        end
    end

    it "should go to exception POSE_TIMEOUT" do

        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        pid.start

        pose_sample = generate_default_pose
        set_point = generate_default_cmd

        pose_samples.write pose_sample
        cmd_in.write set_point
        cmd_out0 = assert_has_one_new_sample cmd_out, 1
        sleep(pid.timeout_in)
        assert_state_change(pid) { |s| s == :POSE_TIMEOUT }
    end

    it "should not crash with just one set_point" do

        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        pid.start

        pose_sample = generate_default_pose
        set_point = generate_default_cmd
        cmd_in.write set_point

        for i in 0..3
            pose_sample.time = Time.now
            puts "pose_sample.time: #{pose_sample.time} + #{pose_sample.time.usec}"
            puts "set_point.time: #{set_point.time} + #{set_point.time.usec}"
            pose_samples.write pose_sample
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            puts "cmd_out0.time: #{cmd_out0.time} + #{cmd_out0.time.usec}"
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.01
            end
            puts " "
        end
    end

end
