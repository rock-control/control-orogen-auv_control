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
            task.addCommandInput 'test', 0
            port = task.cmd_test
            assert_equal '/base/LinearAngular6DCommand_m', port.type.name
        end
        it "should return true when the port could be created" do
            assert task.addCommandInput('test', 0)
        end
        it "should return false when the port already exists" do
            task.addCommandInput('test', 0)
            refute task.addCommandInput('test', 0)
        end

        before do
            task.expected_inputs do |v|
                v.linear[0] = 1
                v.linear[1] = 1
            end
        end

        def invalidated_command
            sample = task.cmd_in.new_sample
            sample.time = Time.now
            sample.linear = Types::Base::Vector3d.Unset
            sample.angular = Types::Base::Vector3d.Unset
            sample
        end

        it "should merge all its inputs" do
            task.addCommandInput '0', 0
            task.addCommandInput '1', 0
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample

            merged = assert_has_one_new_sample cmd_out, 10
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should use the cmd_in port if it is connected" do
            task.addCommandInput '0', 0
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_in.writer
            
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample

            merged = assert_has_one_new_sample cmd_out, 10
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should use the cmd_cascade port if it is connected" do
            task.addCommandInput '0', 0
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_cascade.writer
            
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.linear[1] = 2
            cmd1.write sample

            merged = assert_has_one_new_sample cmd_out, 10
            assert_in_delta 1, merged.linear[0], 1e-6
            assert_in_delta 2, merged.linear[1], 1e-6
            assert Base.unset?(merged.linear[2])
            assert Base.unset?(merged.angular[0])
            assert Base.unset?(merged.angular[1])
            assert Base.unset?(merged.angular[2])
        end
        it "should go in INPUT_MISSING state if an expected input is not there" do
            task.addCommandInput '0', 0
            cmd0 = task.cmd_0.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            assert_state_change(task) { |s| s == :INPUT_MISSING }
        end
        it "should go in INPUT_COLLIDING state if two inputs provide the same value" do
            task.addCommandInput '0', 0
            task.addCommandInput '1', 0
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            sample.linear[1] = Base.unset
            cmd1.write sample
            assert_state_change(task) { |s| s == :INPUT_COLLIDING }
        end
        it "should go in INPUT_UNEXPECTED state if there is a value for an unexpected input" do
            task.addCommandInput '0', 0
            cmd0 = task.cmd_0.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            sample.linear[2] = 1
            cmd0.write sample
            assert_state_change(task) { |s| s == :INPUT_UNEXPECTED }
        end
        it "should go in TIMEOUT state if one port is not updated for its specified timeout" do
            task.addCommandInput '0', 1
            cmd0 = task.cmd_0.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.time = Time.now + 2
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            assert_state_change(task) { |s| s == :TIMEOUT }
        end
        it "should not check for timeouts on ports that have a timeout value of zero" do
            task.addCommandInput '0', 0
            cmd0 = task.cmd_0.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            sample = invalidated_command
            sample.time = Time.now + 1000
            sample.linear[0] = 1
            sample.linear[1] = 1
            cmd0.write sample
            assert_state_change(task) { |s| s == :TIMEOUT }
        end
        it "should wait in WAIT_FOR_INPUT state if there is no data on one of the input ports" do
            task.addCommandInput '0', 0
            task.addCommandInput '1', 0
            cmd0 = task.cmd_0.writer
            cmd1 = task.cmd_1.writer
            task.configure
            task.start
            sample = invalidated_command
            sample.linear[0] = 1
            cmd0.write sample
            assert_state_change(task) { |s| s == :WAIT_FOR_INPUT }
        end

        it "should delete all created ports" do
            task.addCommandInput '0', 0
            task.configure
            task.cleanup
        end
end
