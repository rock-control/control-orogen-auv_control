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

            pose_samples.write pose_sample
            cmd_in.write set_point
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.01
            end
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
        cmd_out0 = assert_has_one_new_sample cmd_out, 0.1
        assert_state_change(pid) { |s| s == :CONTROLLING }
        sleep(pid.timeout_pose.to_i)
        assert_state_change(pid) { |s| s == :POSE_TIMEOUT }
    end

    it "should go to UNSURE_POSE_SAMPLE" do

        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.variance_threshold = 1
        pid.timeout_in = Time.at(10)

        pid.configure
        pid.start

        pose_sample = generate_default_pose
        pose_sample.cov_position.data[0] = 2
        pose_samples.write pose_sample

        set_point = generate_default_cmd

        cmd_in.write set_point
        cmd_out0 = assert_has_one_new_sample cmd_out, 1
        assert_equal pose_sample.time.usec, cmd_out0.time.usec
        assert_state_change(pid) { |s| s == :UNSURE_POSE_SAMPLE }
    end

    it "should not crash with just one set_point" do

        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.timeout_in = Time.at(10)

        pid.configure
        pid.start

        pose_sample = generate_default_pose
        set_point = generate_default_cmd
        cmd_in.write set_point

        for i in 0..3
            pose_sample.time = Time.now
            pose_samples.write pose_sample
            sleep(0.01)
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

    it "should provide the correct output according the pose_sample, considering auv moving along X axis only" do

        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.timeout_in = Time.at(10)
        pid.position_control = true
        pid.world_frame = true
        # The gains other than K are zero
        gainK = pid.pid_settings.linear[0].K

        pid.configure
        pid.start

        # linear pose = (0,0,0)
        pose_sample = generate_default_pose
        # linear set_point = (1,0,0)
        set_point = generate_default_cmd
        cmd_in.write set_point

        for i in 0..3
            pose_sample.time = Time.now
            # Updating position
            pose_sample.position[0] += 0.1*i
            pose_samples.write pose_sample
            sleep(0.01)
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            # Calc expected command output
            command_linX = gainK*(set_point.linear[0] - pose_sample.position[0])
            assert_equal command_linX, cmd_out0.linear[0]

            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

end
