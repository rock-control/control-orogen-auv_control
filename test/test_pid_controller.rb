# frozen_string_literal: true

require "minitest/spec"
require "orocos/test/component"
require "minitest/autorun"

describe "auv_control::PIDController" do
    include Orocos::Test::Component
    start  "pid", "auv_control::PIDController" => "pid"
    reader "pid", "cmd_out", :attr_name => "cmd_out"
    reader "pid", "pid_state", :attr_name => "pid_state"
    writer "pid", "cmd_in", :attr_name => "cmd_in"
    writer "pid", "pose_samples", :attr_name => "pose_samples"

    def quaternion_from_euler(rpy)
        # Euler angles respecting ZYX order
        Eigen::Quaternion.from_euler(
            Eigen::Vector3.new(rpy[2], rpy[1], rpy[0]), 2, 1, 0
        )
    end

    def vector3d_from_array(array)
        Eigen::Vector3.new(array[0], array[1], array[2])
    end

    def generate_rbs(
        pos = [0, 0, 0], rpy = [0, 0, 0], lin_vel = [0, 0, 0], ang_vel = [0, 0, 0]
    )
        pose_sample = pid.pose_samples.new_sample
        pose_sample.position = vector3d_from_array(pos)
        pose_sample.orientation = quaternion_from_euler(rpy)
        pose_sample.velocity = vector3d_from_array(lin_vel)
        pose_sample.angular_velocity = vector3d_from_array(ang_vel)
        pose_sample.time = Time.now
        pose_sample
    end

    def generate_cmd(pos = [1, 0, 0], rpy = [0, 0, 0])
        set_point = pid.cmd_in.new_sample
        set_point.linear = vector3d_from_array(pos)
        set_point.angular = vector3d_from_array(rpy)
        set_point.time = Time.now
        set_point
    end

    it "should not provide output samples with same timestamp" do
        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        pid.start

        pose_sample = generate_rbs
        set_point = generate_cmd

        cmd_in.write set_point

        3.times do
            pose_sample.time = Time.now
            set_point.time = Time.now

            pose_samples.write pose_sample
            cmd_in.write set_point
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            5.times do
                # No more repeated sample here.
                assert_has_no_new_sample cmd_out, 0.01
            end
        end
    end

    it "should go to exception POSE_TIMEOUT" do
        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        pid.start

        pose_sample = generate_rbs
        set_point = generate_cmd

        pose_samples.write pose_sample
        cmd_in.write set_point
        assert_has_one_new_sample cmd_out, 0.1
        assert_state_change(pid) { |s| s == :CONTROLLING }
        sleep(pid.timeout_pose.to_i)
        assert_state_change(pid) { |s| s == :POSE_TIMEOUT }
    end

    it "should not go to exception POSE_TIMEOUT" do
        pid.apply_conf_file("auv_control::PIDController.yml")

        pid.configure
        sleep(pid.timeout_pose.to_i)
        pid.start

        pose_sample = generate_rbs
        set_point = generate_cmd

        sleep(0.1)

        pose_samples.write pose_sample
        cmd_in.write set_point
        assert_has_one_new_sample cmd_out, 0.1
        assert_state_change(pid) { |s| s == :CONTROLLING }
    end

    it "should go to UNSURE_POSE_SAMPLE" do
        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.variance_threshold = 1
        pid.timeout_in = Time.at(10)

        pid.configure
        pid.start

        pose_sample = generate_rbs
        pose_sample.cov_position.data[0] = 2
        pose_samples.write pose_sample

        set_point = generate_cmd

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

        pose_sample = generate_rbs
        set_point = generate_cmd
        cmd_in.write set_point

        3.times do
            pose_sample.time = Time.now
            pose_samples.write pose_sample
            sleep(0.01)
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            5.times do
                # No more repeated sample here.
                assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

    it "should provide the correct output according the pose_sample, considering auv \
        moving along X axis only" do
        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.timeout_in = Time.at(10)
        pid.position_control = true
        pid.world_frame = true
        # The gains other than K are zero
        gain_k = pid.pid_settings.linear[0].K

        pid.configure
        pid.start

        # linear pose = (0,0,0)
        pose_sample = generate_rbs
        # linear set_point = (1,0,0)
        set_point = generate_cmd
        cmd_in.write set_point

        (0..3).each do |i|
            pose_sample.time = Time.now
            # Updating position
            pose_sample.position[0] += 0.1 * i
            pose_samples.write pose_sample
            sleep(0.01)
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal pose_sample.time.usec, cmd_out0.time.usec
            # Calc expected command output
            command_lin_x = gain_k * (set_point.linear[0] - pose_sample.position[0])
            assert_equal command_lin_x, cmd_out0.linear[0]

            5.times do
                # No more repeated sample here.
                assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

    it "should provide roll, pitch and yaw errors normalized between -pi and pi" do
        pid.apply_conf_file("auv_control::PIDController.yml")
        pid.timeout_in = Time.at(10)
        pid.position_control = true
        pid.world_frame = true

        pid.configure
        pid.start

        # Initial Euler angles
        # Yaw is on its lower limit to force pi = -pi discontinuity
        roll = 0.0
        pitch = 0.0
        yaw = -Math::PI

        # Setpoints respecting Euler angles limits
        # Yaw is on its upper limit to force pi = -pi discontinuity
        roll_sp = Math::PI / 4
        pitch_sp = -Math::PI / 4
        yaw_sp = Math::PI

        pose_sample = generate_rbs([0, 0, 0], [roll, pitch, yaw])
        set_point = generate_cmd([0, 0, 0], [roll_sp, pitch_sp, yaw_sp])
        cmd_in.write set_point

        (1..10).each do |i|
            pose_sample.time = Time.now
            pose_samples.write pose_sample
            sleep(0.01)
            out_pid_state = assert_has_one_new_sample pid_state, 1

            # Asserts that all calculated errors are between -pi and pi
            (0..2).each do |j|
                # As angular P-Controllers have K = 1, then rawOutput = error
                assert_operator out_pid_state.angular[j].rawOutput, :>=, -Math::PI
                assert_operator out_pid_state.angular[j].rawOutput, :<=,  Math::PI
            end

            # Updating orientation
            roll  += 1.0 / (2 * i)
            pitch -= 1.0 / (2 * i)
            yaw   += 1.0 / (2 * i)
            pose_sample.orientation = quaternion_from_euler([roll, pitch, yaw])
        end
    end
end
