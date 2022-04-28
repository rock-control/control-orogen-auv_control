require 'minitest/autorun'
require 'minitest/spec'
require 'orocos/test/component'

describe 'auv_control::Base' do
    include Orocos::Test::Component

    # We cannot instanciate an auv_control::Base component, as it is abstract.
    # Instead, use WorldToAligned to test the Base functionality
    start 'task', 'auv_control::WorldToAligned' => 'task'
    reader 'task', 'cmd_out', :attr_name => 'cmd_out'

        it "should create a port with the given name prefixed with cmd_" do
            task.addCommandInput('test', Time.at(0))
            port = task.cmd_test
            assert_equal '/base/commands/LinearAngular6DCommand_m', port.type.name
        end
        it "should return true when the port could be created" do
            assert task.addCommandInput('test', Time.at(0))
        end
        it "should return false when the port already exists" do
            task.addCommandInput('test', Time.at(0))
            refute task.addCommandInput('test', Time.at(0))
        end

        before do
            task.expected_inputs do |v|
                v.linear = [true, true, false]
                v.angular = [false, false, false]
            end
        end

        def invalidated_command
            sample = task.cmd_in.new_sample
            sample.time = Time.now
            sample.linear = Eigen::Vector3.Unset
            sample.angular = Eigen::Vector3.Unset
            sample
        end

        def pose_sample
            sample = task.pose_samples.new_sample
            sample.time = Time.now
            sample.position = sample.velocity = sample.angular_velocity = Eigen::Vector3.new(0, 0, 0)
            sample.orientation = Eigen::Quaternion.Identity
            sample
        end

        it "should merge all its inputs" do
            task.addCommandInput '0', Time.at(0)
            task.addCommandInput '1', Time.at(0)
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample
            pose.write pose_sample

            merged = assert_has_one_new_sample cmd_out, 1
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should use the cmd_in port if it is connected" do
            task.addCommandInput '0', Time.at(0)
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_in.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample
            pose.write pose_sample

            merged = assert_has_one_new_sample cmd_out, 10
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should use the cmd_cascade port if it is connected" do
            task.addCommandInput '0', Time.at(0)
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_cascade.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample
            pose.write pose_sample

            merged = assert_has_one_new_sample cmd_out, 10
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should go in INPUT_MISSING state if an expected input is not there" do
            task.addCommandInput '0', Time.at(0)
            cmd0 = task.cmd_0.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            pose.write pose_sample
            cmd0.write sample
            assert_state_change(task) { |s| s == :INPUT_MISSING }
        end
        it "should go in INPUT_COLLIDING state if two inputs provide the same value" do
            task.addCommandInput '0', Time.at(0)
            task.addCommandInput '1', Time.at(0)
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            pose.write pose_sample
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            sample.linear[1] = Base.unset
            cmd1.write sample
            assert_state_change(task) { |s| s == :INPUT_COLLIDING }
        end
        it "should go in INPUT_UNEXPECTED state if there is a value for an unexpected input" do
            task.addCommandInput '0', Time.at(0)
            cmd0 = task.cmd_0.writer
            pose = task.pose_samples.writer

            task.keep_position_on_exception = false
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            sample.linear[2] = 1
            cmd0.write sample
            pose.write pose_sample
            assert_state_change(task) { |s| s == :INPUT_UNEXPECTED }
        end
        it "should go in TIMEOUT state if one port is not updated for its specified timeout and recover in case of new data" do
            task.addCommandInput '0', Time.at(1)
            cmd0 = task.cmd_0.writer
            pose = task.pose_samples.writer
            task.timeout_pose = Time.at(10)
            task.keep_position_on_exception = false

            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1


            cmd0.write sample
            pose.write pose_sample
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_equal sample.time.usec, cmd_out0.time.usec
            sample = invalidated_command
            cmd_out0 = assert_has_no_new_sample cmd_out, 1
            sleep(1)
            assert_state_change(task) { |s| s == :TIMEOUT }

            sample.time = Time.now
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            sleep(0.02)
            cmd_out0 = assert_has_one_new_sample cmd_out, 1
            assert_state_change(task) { |s| s == :CONTROLLING }
        end
        it "should not check for timeouts on ports that have a timeout value of zero" do
            task.addCommandInput '0', Time.at(0)
            cmd0 = task.cmd_0.writer
            pose = task.pose_samples.writer
            task.timeout_pose = Time.at(10)
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            pose.write pose_sample
            sample = invalidated_command
            sample.time = Time.now
            sleep(2)
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            pose.write pose_sample
            assert_state_change(task) { |s| s != :TIMEOUT }
        end

        it "should control if multiple inputs are connected, with only producing all the expected data" do
            task.addCommandInput '0', Time.at(0)
            task.addCommandInput '1', Time.at(0)
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            pose = task.pose_samples.writer

            task.expected_inputs = Hash[linear: [true, true, false], angular: [false, false, false]]
            task.keep_position_on_exception = false
            task.configure
            task.start

            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            pose.write pose_sample
            assert_state_change(task) { |s| s == :CONTROLLING }
        end

        it "should delete all created ports" do
            task.addCommandInput '0', Time.at(0)
            task.configure
            task.cleanup
        end

        it "should go to WAIT_FOR_CONNECTED_INPUT_PORT state when no port is connected" do
            pose = task.pose_samples.writer
            task.keep_position_on_exception = false
            task.configure
            task.start
            pose.write pose_sample
            assert_state_change(task) { |s| s == :WAIT_FOR_CONNECTED_INPUT_PORT }
        end

        it "should NOT go to WAIT_FOR_CONNECTED_INPUT_PORT state when at least one port is connected" do
            pose = task.pose_samples.writer
            cmd = task.cmd_in.writer
            task.configure
            task.start
            pose.write pose_sample
            assert_state_change(task) { |s| s != :WAIT_FOR_CONNECTED_INPUT_PORT }
        end
end
