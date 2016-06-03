require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'


describe 'auv_control::ThrustersFeedback configuration' do
  include Orocos::Test::Component
  start  'thrusters_feedback', 'auv_control::ThrustersFeedback' => 'thrusters_feedback'
  reader 'thrusters_feedback', 'cmd_out', :attr_name => 'cmd_out'
  writer 'thrusters_feedback', 'cmd_in', :attr_name => 'cmd_in'

  it 'thruster_coeff_pos < 0' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")

    aux = thrusters_feedback.thruster_coeff_pos
    aux[0] = -8
    thrusters_feedback.thruster_coeff_pos = aux
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'thruster_coeff_neg < 0' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")

    aux = thrusters_feedback.thruster_coeff_neg
    aux[0] = -8
    thrusters_feedback.thruster_coeff_neg = aux
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'wrong control_modes size' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'control_mode_3_raw'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'wrong thruster_coeff_pos size' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'thruster_coeff_pos_size_3'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'automatically setting control_modes' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    aux = thrusters_feedback.control_modes
    aux.clear
    thrusters_feedback.control_modes = aux 
      
    thrusters_feedback.configure
  end

  it 'pos and neg coefficients have different sizes' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'thruster_coeff_pos_size_3'], true)
    
    aux = thrusters_feedback.control_modes
    aux.clear
    thrusters_feedback.control_modes = aux 
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'control modes set to EFFORT' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'control_modes_effort'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'control mode equals to RAW but no thruster voltage was set' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    thrusters_feedback.thruster_voltage = 0
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_feedback.configure }
  end

  it 'wrong number of thrusters input' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    thrusters_feedback.configure  
    thrusters_feedback.start  
      
    sample = thrusters_feedback.cmd_in.new_sample
    
    # 3 thrusters inputs were sent and 2 are expected
    thruster = Types::Base::JointState.new
    thruster.effort = 3
    sample.elements = [thruster, thruster, thruster]
    cmd_in.write sample

    assert_state_change(thrusters_feedback) { |s| s == :UNEXPECTED_THRUSTER_INPUT } 
        
  end
  
  it 'thruster input not set' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    thrusters_feedback.configure  
    thrusters_feedback.start  

    sample = thrusters_feedback.cmd_in.new_sample
      
    # the effort field was not set
    thruster = Types::Base::JointState.new
    sample.elements = [thruster, thruster]
    cmd_in.write sample

    assert_state_change(thrusters_feedback) { |s| s == :UNSET_THRUSTER_INPUT } 
        
  end
  
  it 'testing positive speed calculated value' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'control_modes_speed'], true)
    
    thrusters_feedback.configure  
    thrusters_feedback.start  

    sample = thrusters_feedback.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.speed = 457
    thruster2.speed = 789
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].effort - 13.99).abs < 0.001
    assert (data.elements[1].effort - 41.701).abs < 0.001
        
  end

  it 'testing negative speed calculated value' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml",['default', 'control_modes_speed'], true)
    
    thrusters_feedback.configure  
    thrusters_feedback.start  

    sample = thrusters_feedback.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.speed = -530
    thruster2.speed = -650
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].effort + 18.817).abs < 0.001
    assert (data.elements[1].effort + 28.302).abs < 0.001
        
  end

  it 'testing positive raw calculated value' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    thrusters_feedback.configure  
    thrusters_feedback.start  

    sample = thrusters_feedback.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.raw = 25
    thruster2.raw = 17
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].effort - 15.114).abs < 0.001
    assert (data.elements[1].effort - 6.9887).abs < 0.001
  end

  it 'testing negative raw calculated value' do

    thrusters_feedback.apply_conf_file("auv_control::ThrustersFeedback.yml")
    
    thrusters_feedback.configure  
    thrusters_feedback.start  

    sample = thrusters_feedback.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.raw = -49
    thruster2.raw = -31
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].effort + 58.061).abs < 0.001
    assert (data.elements[1].effort + 23.239).abs < 0.001
  end

end

