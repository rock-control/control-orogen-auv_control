require "minitest/spec"
require "orocos/test/component"
require "minitest/autorun"

describe "auv_control::WorldToAligned" do
    include Orocos::Test::Component
    start  "world_to_aligned", "auv_control::WorldToAligned" => "world_to_aligned"
    reader "world_to_aligned", "cmd_out", :attr_name => "cmd_out"
    writer "world_to_aligned", "cmd_in", :attr_name => "cmd_in"
    writer "world_to_aligned", "pose_samples", :attr_name => "pose_samples"

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
        pose_sample = world_to_aligned.pose_samples.new_sample
        pose_sample.position = vector3d_from_array(pos)
        pose_sample.orientation = quaternion_from_euler(rpy)
        pose_sample.velocity = vector3d_from_array(lin_vel)
        pose_sample.angular_velocity = vector3d_from_array(ang_vel)
        pose_sample.time = Time.now
        pose_sample
    end

    def generate_cmd(lin = [1, 0, 0], ang = [0, 0, 0])
        set_point = world_to_aligned.cmd_in.new_sample
        set_point.linear = vector3d_from_array(lin)
        set_point.angular = vector3d_from_array(ang)
        set_point.time = Time.now
        set_point
    end

    it "should create a port with the given name prefixed with cmd_" do
        world_to_aligned.addCommandInput "test", Time.at(0)
        port = world_to_aligned.cmd_test
        assert_equal "/base/commands/LinearAngular6DCommand_m", port.type.name
    end

    it "should not provide output samples with same timestamp" do
        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_rbs
        cmd = generate_cmd

        3.times do
            pose_sample.time = Time.now
            cmd.time = Time.now
            pose_samples.write pose_sample
            cmd_in.write cmd
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal cmd.time.usec, cmd_out0.time.usec
            4.times do
                # No more repeated sample here.
                assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

    it "should go to exception POSE_TIMEOUT" do
        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")
        world_to_aligned.timeout_in = Time.at(world_to_aligned.timeout_pose.to_i + 1)
        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_rbs
        cmd = generate_cmd

        pose_samples.write pose_sample
        cmd_in.write cmd
        assert_has_one_new_sample cmd_out, 0.1
        # sleep(world_to_aligned.timeout_in)
        cmd_in.write generate_cmd
        assert_state_change(world_to_aligned) { |s| s == :CONTROLLING }
        assert_has_one_new_sample cmd_out, 0.1
        sleep(world_to_aligned.timeout_pose.to_i)
        assert_has_no_new_sample cmd_out, 0.1
        assert_state_change(world_to_aligned) { |s| s == :POSE_TIMEOUT }
    end

    it "should not crash with just one pose_sample" do
        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_rbs
        cmd = generate_cmd
        pose_samples.write pose_sample
        cmd_in.write cmd

        3.times do
            cmd.time = Time.now
            cmd_in.write cmd
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal cmd.time.usec, cmd_out0.time.usec
            4.times do
                # No more repeated sample here.
                assert_has_no_new_sample cmd_out, 0.01
            end
        end
    end

    it "should provide output samples even in case there is one sample in input_port" do
        cmd_cascade = world_to_aligned.cmd_cascade.writer

        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_rbs
        cmd = world_to_aligned.cmd_in.new_sample
        cmd.linear = Eigen::Vector3.Zero
        cmd.angular = Eigen::Vector3.Unset
        cmd.time = Time.now
        cmd_c = world_to_aligned.cmd_in.new_sample
        cmd_c.linear = Eigen::Vector3.Unset
        cmd_c.angular = Eigen::Vector3.Zero
        cmd_c.time = Time.now
        sleep(0.01)
        assert_state_change(world_to_aligned) { |s| s == :WAIT_FOR_POSE_SAMPLE }

        3.times do
            pose_sample.time = Time.now
            cmd.time = Time.now
            cmd_c.time = Time.now
            pose_samples.write pose_sample
            cmd_in.write cmd
            cmd_cascade.write cmd_c
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            # The latest command
            assert_equal cmd_c.time.usec, cmd_out0.time.usec
            4.times do
                # No more repeated sample here.
                cmd_c.time = Time.now
                cmd_cascade.write cmd_c
                cmd_out1 = assert_has_one_new_sample cmd_out, 0.1
                assert_equal cmd_c.time.usec, cmd_out1.time.usec
            end
        end
    end

    it "should consider the angular velocity representation to calculate the command" do
        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        roll = -Math::PI / 6
        pitch = Math::PI / 6
        yaw = Math::PI / 2

        ang_vel_cmd = [1, 1, 1]

        # By default, the axis-angle representation is assumed. No modification should
        # be done into the angular velocity command, regardless the current pose.
        pose_sample = generate_rbs([1, 2, 3], [roll, pitch, yaw], [0, 0, 0], [0, 0, 0])
        cmd = generate_cmd([1, 2, 3], ang_vel_cmd)

        pose_sample.time = Time.now
        cmd.time = Time.now

        pose_samples.write pose_sample
        cmd_in.write cmd
        sample = assert_has_one_new_sample cmd_out, 1

        expected_ang_vel = cmd.angular
        assert_equal expected_ang_vel, sample.angular

        # Assures that nothing is done when position_control = true even when
        # ang_vel_euler_rate is set to true
        world_to_aligned.position_control = true
        world_to_aligned.ang_vel_euler_rate = true

        pose_samples.write pose_sample
        cmd_in.write cmd
        sample = assert_has_one_new_sample cmd_out, 1

        # Should have the same angular[0] and angular[1] values, angular[2] is altered
        # as by removing the yaw value
        expected_ang_vel = cmd.angular

        assert_equal expected_ang_vel[0], sample.angular[0]
        assert_equal expected_ang_vel[1], sample.angular[1]
        assert_in_delta expected_ang_vel[2] - yaw, sample.angular[2], 1e-3

        # This should change the output angular velocity command to consider an axis-angle
        # representation as expected and not an Euler rate as provided.
        world_to_aligned.position_control = false

        pose_samples.write pose_sample
        cmd_in.write cmd
        sample = assert_has_one_new_sample cmd_out, 1

        # Expected result should be given by w = J_inv(roll, pitch) * euler_rate
        expected_ang_vel = vector3d_from_array([0.5, 0.433, 1.25])
        (0..3).each do |i|
            assert_in_delta expected_ang_vel[i], sample.angular[i], 1e-3
        end
    end
end
