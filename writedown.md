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
        -  _mode 、 _currentState 和 _nextState 这三个对象在 initialize() 中进行初始化， initialize() 在构造函数中使用。
        - 构造函数内初始化了 _stateList 对象，该对象在 initialize() 中用于初始化 _currentState 和 _nextState 。 _currentState 初始被赋予为walking状态。关于_stateList在后面会提到。
        - _mode 在 initialize() 中被赋予为NORMAL模式，这个模式是状态机切换的标志位，将在 run() 中用于逻辑判断。
      - run()
        - 主要实现三个功能：安全性判断，数据交互，状态机检测与切换。
        - 数据交换：_data->sendRecv() 在对象 CheaIO::sendRecv 中实现，是一个多层封装的使用
        - 安全性判断：通过FSM类的checkSafty对象实现，并通过 _data->_interface->setPassive()对象实现对机器人的失能控制
        - 状态机检测与切换：接口是FSMState的虚函数checkTransition(),具体实现借助_lowState->userCmd的数据来进行判断。
    - 内部数据存放了一系列用于状态机检测与切换的数据对象和函数对象。
      - _data 存放了所有的数据结构，以参数传入的形式进行初始化
      - _currentState 和 _nextState 是状态机用来切换的载体，