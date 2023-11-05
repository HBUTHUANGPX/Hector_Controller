hector_control

main.cpp 为主程序
在main函数中实例化多个对象，并合并于ControlFSMData类的_controlData对象这一数据集合中，并通过该数据合集初始化FSM的对象，FSM类集成了数据结构和状态机。
主循环调用FSM的run()对象。

src/FSM/*.cpp

FSM.cpp/FSM.h 
- 实现了FSM类和FSMStateList结构体
  - FSM类
    - 外部接口主要实现了状态机的初始化 initialize()和运行run()
      - initialize()
        -  _mode 、 _currentState 和 _nextState 这三个变量在函数 initialize() 中进行初始化，函数 initialize() 在构造函数中使用。
        - 构造函数内初始化了 _stateList 变量，该变量在函数 initialize() 中用于初始化变量 _currentState 和 _nextState 。变量 _currentState 初始被赋予为walking状态。关于变量 _stateList 在后面会提到。
        - _mode 在函数 initialize() 中被赋予为NORMAL模式，这个模式是状态机切换的标志位，将在函数 run() 中用于逻辑判断。
      - run()
        - 主要实现四个功能：安全性判断，数据交互，状态机检测与切换，状态机执行。
        - 数据交换：_data->sendRecv() 在函数 CheaIO::sendRecv 中实现，是一个多层封装的使用
        - 安全性判断：通过FSM类的checkSafty函数实现，并通过 _data->_interface->setPassive() 函数实现对机器人的失能控制
        - 状态机检测与切换：接口是FSMState的虚函数checkTransition(),具体实现借助 _lowState->userCmd 变量的数据来进行判断。
        - 状态机执行：使用_currentState的run()函数。
    - 内部数据存放了一系列用于状态机检测与切换的数据对象和函数对象。
      - _data 存放了所有的数据结构，以参数传入的形式进行初始化
      - _currentState 和 _nextState 是状态机用来切换的载体。正常情况主要是使用 _currentState的run()函数。若是进行状态切换将会调用exit()和enter()。
      - _nextStateName 通过 _currentState 的checkTransition() 成员函数进行刷新，随后与 _currentState 的 _stateName 进行比较判断，若两者不相同则将 _mdoe 切换为CHANGE状态，并将 _nextState 通过FSM类的 getNextState()函数进行刷新
      - _mode 是 FSMMode类的实现，FSMMode是一个enum class，其中包含了两个状态，分别为CHANGE和NORMAL
      - _stateList 是对 FSMStateList 结构体的具体实现。
  - FSMStateList
    - 内部主要是三个变量，分别是 invalid passive walking。这三个变量分别是 FSMState FSMState_Passive FSMState_Walking 的具体实现，后两个类是对前一个类是继承关系。

FSMState*.cpp/FSMState*.h
- FSMState_Walking内实现了一个重新映射函数 invNormalize()，用来把变量的值映射在新的定义域内。
- FSMState 类初步实现了函数接口，FSMState_Passive和 FSMState_Walking 对 enter()、run()、exit()、checkTransition() 四个函数进行了具体实现
  - FSMState_Walking
    - 内部数据存放了 Cmpc、v_des_body、turn_rate、pitch、roll
      - v_des_body是机器人质心的速度
      - turn_rate 是机器人的yaw轴角速度
      - r、p是机器人roll轴、pitch轴的角度
        - 这些数据对应论文里的公式(7)的1*12向量【Θ pc ω pc`】
      - Cmpc 是 ConvexMPCLocomotion类的实现，后面再讲，是关键部分
    - run()
      1. 开头和结尾分别调用 _data->_legController 的 updateData() 和 updateCommand()函数。前者计算腿的雅可比、位置以及速度，后者用来更新cmd。具体在 LegController.cpp 讲述。
      2. 随后调用 _data->_stateEstimator 的 run()函数 
          - run()的具体实现于StateEstimatorContainer类的run()中,这是一个通过vector实现的容器类，该容器存放了具体的Estimator。而真正具体的run()分别位于 CheaterPositionVelocityEstimator和 CheaterOrientationEstimator两个类中，这两个类是继承于 GenericEstimator 类。 _stateEstimator 在main函数中基于 StateEstimatorContainer 类进行实例化，这个类中的变量 _estimators 是使用vector 基于 GenericEstimator类 构建。
          - CheaterOrientationEstimator::run()
            - result->orientation通过lowState->imu.quaternion赋值
            - result->rBody通过result->orientation进行四元数转旋转矩阵获得
            - result->omegaBody通过lowState->imu.gyroscope赋值(角速度)
            - result->rpy通过result->orientation进行四元数转欧拉角获得
            - result->omegaWorld通过result->rBody的转置乘result->omegaBody获得
          - CheaterPositionVelocityEstimator::run()
            - result->position 通过 lowState->position 赋值
            - result->vWorld 通过 lowState->vWorld赋值
            - result->vBody 通过 result->rBody 乘 result->vWorld 获得
      3. 然后更新 _userValue 变量，并使用 invNormalize() 对 _userValue的 ly、rx、lx进行重映射，并且赋值给v_des_body[0],v_des_body[1],turn_rate
      4. 随后 _data->_desiredStateCommand->setStateCommands 通过 传入的变量以及 第2步更新的stateEstimate变量 对 stateDes 和 pre_stateDes 变量进行 赋值。stateDes将在后面使用。
      5. 最后使用Cmpc的run()函数。
    - 值得注意的是 _userValue的赋值来源 _data->_lowState->userValue 是在 CheaIO::sendRecv中进行 ，使用 cmdPanel->getUserValue()

ConvexMPC
ConvexMPCLocomotion.cpp/.h
- 


修改参数的地方：

solverMPC.cpp
  488/489/490行 mu是摩擦系数，lt是l toe，lh是l heel
  原文后两个参数是 0.09，0.05
  代码后两个参数是 0.09，0.06

Constraints.cpp
  128/129/130行 mu是摩擦系数，lt是l toe，lh是l heel
  原文后两个参数是 0.09，0.05
  代码后两个参数是 0.09，0.06

Biped.h
  12-22行
    leg_offset_x = 0.0;
    leg_offset_y = 0.047;
    leg_offset_z = -0.1360;
    leg_offset_x2 = 0.0;
    leg_offset_y2 = 0.047;
    leg_offset_z2 = -0.136;
    hipLinkLength = 0.038; // hip offset in const.xacro
    thighLinkLength = 0.22;
    calfLinkLength = 0.22;

ConvexMPCLocomotion.cpp
  53行 world_position_desired[2] = 0.55;
  89行 pBody_des[2] = 0.55;
  这个应该是机器人初始的那个高度
  214行 Vec3<double> hipOffset = {0, side * -0.02, -0.136};
  297-303 行 
    q(2) += 0.3 * PI;
    q(3) -= 0.6 * PI;
    q(4) += 0.3 * PI;

    q(7) += 0.3 * PI;
    q(8) -= 0.6 * PI;
    q(9) += 0.3 * PI;
  351行 0.55
LegController.cpp
  111-113
    q(2) = q(2) + 0.3*3.14159;
    q(3) = q(3) - 0.6*3.14159;
    q(4) = q(4) + 0.3*3.14159;
