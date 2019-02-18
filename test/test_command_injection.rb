require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'auv_control::PIDController' do
    include Orocos::Test::Component

    start  'injection', 'auv_control::CommandInjection' => 'injection'
    reader 'injection', 'cmd_out', attr_name: 'cmd_out'
    writer 'injection', 'cmd_injection', attr_name: 'cmd_injection'

    before do
        injection.expected_inputs = Hash[linear: [true, true, false], angular: [false, false, false]]
        injection.addCommandInput 'controller', Time.at(0)
        injection.configure
        injection.start
    end

    it "outputs the injected command if it matches the expectations regardless of the other input states" do
        injected_cmd = Types.base.commands.LinearAngular6DCommand.new(
            time: Time.now, linear: Eigen::Vector3.new(1, 2, 0), angular: Eigen::Vector3.Zero)

        w = injection.cmd_injection.writer
        w.write injected_cmd
        sample = assert_has_one_new_sample cmd_out
        assert_equal 1, sample.linear[0]
        assert_equal 2, sample.linear[1]
        assert Base.unknown?(sample.linear[2])
        3.times { |i| assert Base.unknown?(sample.angular[i]) }
    end
end

