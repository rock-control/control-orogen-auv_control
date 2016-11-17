require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'auv_control::WorldToAligned' do
    include Orocos::Test::Component
    start  'world_to_aligned', 'auv_control::WorldToAligned' => 'world_to_aligned'
    reader 'world_to_aligned', 'cmd_out', :attr_name => 'cmd_out'
    writer 'world_to_aligned', 'cmd_in', :attr_name => 'cmd_in'
    writer 'world_to_aligned', 'pose_samples', :attr_name => 'pose_samples'

    def generate_default_pose
        pose_sample = world_to_aligned.pose_samples.new_sample
        pose_sample.position = pose_sample.velocity = pose_sample.angular_velocity = Eigen::Vector3.new(0, 0, 0)
        pose_sample.orientation = Eigen::Quaternion.Identity
        pose_sample.time = Time.now
        pose_sample
    end

    def generate_default_cmd
        cmd_in = world_to_aligned.cmd_in.new_sample
        cmd_in.linear = Eigen::Vector3.new(1, 0, 0)
        cmd_in.angular = Eigen::Vector3.new(0, 0, 0)
        cmd_in.time = Time.now
        cmd_in
    end

    it "should create a port with the given name prefixed with cmd_" do
        world_to_aligned.addCommandInput 'test', Time.at(0)
        port = world_to_aligned.cmd_test
        assert_equal '/base/commands/LinearAngular6DCommand_m', port.type.name
    end

    it "should not provide output samples with same timestamp" do

        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_default_pose
        cmd = generate_default_cmd

        for i in 0..3
            pose_sample.time = Time.now
            cmd.time = Time.now
            pose_samples.write pose_sample
            cmd_in.write cmd
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal cmd.time.usec, cmd_out0.time.usec
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.1
            end
        end
    end

    it "should go to exception POSE_TIMEOUT" do

        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")
        world_to_aligned.timeout_in = Time.at(world_to_aligned.timeout_pose.to_i+1)
        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_default_pose
        cmd = generate_default_cmd

        pose_samples.write pose_sample
        cmd_in.write cmd
        cmd_out0 = assert_has_one_new_sample cmd_out, 0.1
        # sleep(world_to_aligned.timeout_in)
        cmd_in.write generate_default_cmd
        assert_state_change(world_to_aligned) { |s| s == :CONTROLLING }
        cmd_out0 = assert_has_one_new_sample cmd_out, 0.1
        sleep(world_to_aligned.timeout_pose.to_i)
        cmd_out0 = assert_has_no_new_sample cmd_out, 0.1
        assert_equal :POSE_TIMEOUT, world_to_aligned.state_reader.read
    end

    it "should not crash with just one pose_sample" do

        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_default_pose
        cmd = generate_default_cmd
        pose_samples.write pose_sample
        cmd_in.write cmd

        for i in 0..3
            cmd.time = Time.now
            cmd_in.write cmd
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal cmd.time.usec, cmd_out0.time.usec
            for j in 1..5
                # No more repeated sample here.
                cmd_out1 = assert_has_no_new_sample cmd_out, 0.01
            end
        end
    end

    it "should provide output samples even in case there is one sample in input_port" do
        cmd_cascade = world_to_aligned.cmd_cascade.writer

        world_to_aligned.apply_conf_file("auv_control::WorldToAligned.yml")

        world_to_aligned.configure
        world_to_aligned.start

        pose_sample = generate_default_pose
        cmd = world_to_aligned.cmd_in.new_sample
        cmd.angular = Eigen::Vector3.Unset
        cmd.time = Time.now
        cmd_c = world_to_aligned.cmd_in.new_sample
        cmd_c.linear = Eigen::Vector3.Unset
        cmd_c.time = Time.now
        sleep(0.01)
        assert_equal :WAIT_FOR_POSE_SAMPLE, world_to_aligned.state_reader.read

        for i in 0..3
            pose_sample.time = Time.now
            cmd.time = Time.now
            cmd_c.time = Time.now
            pose_samples.write pose_sample
            cmd_in.write cmd
            cmd_cascade.write cmd_c
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            # The latest command
            assert_equal cmd_c.time.usec, cmd_out0.time.usec
            for j in 1..5
                # No more repeated sample here.
                cmd_c.time = Time.now
                cmd_cascade.write cmd_c
                cmd_out1 = assert_has_one_new_sample cmd_out, 0.1
                assert_equal cmd_c.time.usec, cmd_out1.time.usec
            end
        end
    end

end
