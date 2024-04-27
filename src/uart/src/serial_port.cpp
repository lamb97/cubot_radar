//
// Created by plutoli on 2021/7/19.
//

#include "serial_port.h"

// ******************************  SerialPort类的公有函数  ******************************

// 构造函数

SerialPort::SerialPort():
    param_(),
    isInitialized_(false),
    isOpened_(false),
    fileDescriptor_(-1),
    initTimestamp_(0),
    openTimestamp_(0),
    operateMutex_(),
    readBuffer_(nullptr),
    writeBuffer_(nullptr),
    bytesToRead_(0),
    bytesToWrite_(0),
    readMutex_(),
    writeMutex_(),
    readSwitch_(false),
    writeSwitch_(false),
    readFromDriverThread_(),
    writeToDriverThread_(),
    writeConditionVariable_(),
    dataReceivedHandler_(nullptr),
    dataReceivedUserData_(nullptr)
{
}

// 析构函数
SerialPort::~SerialPort()
{
    // 如果串口处于打开状态，则关闭串口
    if (isOpened_)
    {
        Close();
    }

    // 如果串口资源没有释放，则释放串口资源
    if (isInitialized_)
    {
        Release();
    }

    // 解除注册串口接收数据回调函数
    UnregistDataReceivedHandler();
}

// 获取串口配置参数
SerialPortParam SerialPort::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置串口配置参数
bool SerialPort::SetParam(const SerialPortParam &param)
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经初始化
    if (isInitialized_)
    {
        spdlog::error("SerialPortParam was set failure because it has been Initialized");
        spdlog::info(LOG_END);
        return false;
    }
    // 记录串口配置参数
    param_ = param;

    // 记录日志信息
    spdlog::info("[" + param_.Key + "] - SerialPortParam was set successful");
    spdlog::info(LOG_END);


    // 返回设置结果
    return true;
}

// 获取串口的初始化状态
bool SerialPort::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取串口的打开状态
bool SerialPort::IsOpened()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isOpened_;
}

// 获取串口的初始化时间戳
uint64_t SerialPort::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 获取串口的打开时间戳
uint64_t SerialPort::GetOpenTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return openTimestamp_;
}

// 初始化串口
bool SerialPort::Init()
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);


    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经打开
    if (isOpened_)
    {
        spdlog::error("[" + param_.Key + "] - SerialPort was Initialized failure because it has been opened");
        spdlog::info(LOG_END);
        return false;
    }

    // 判断串口是否已经初始化
    if (isInitialized_)
    {
        spdlog::warn("[" + param_.Key + "] - SerialPort can not be initialized repeatedly");
        spdlog::info(LOG_END);
        return false;
    }

    // 创建读取缓冲区，初始化缓冲区中的字节数
    readBuffer_ = new unsigned char[param_.ReadBufferSize];
    ::memset(readBuffer_, 0, param_.ReadBufferSize);
    bytesToRead_ = 0;

    // 创建写入缓冲区，初始化缓冲区中的字节数
    writeBuffer_ = new unsigned char[param_.WriteBufferSize];
    ::memset(writeBuffer_, 0, param_.WriteBufferSize);
    bytesToWrite_ = 0;

    // 设置串口的初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    spdlog::info("SerialPort was initialized successful");
    spdlog::info(LOG_END);
    // 返回初始化结果
    return true;
}

// 释放串口资源
bool SerialPort::Release()
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经打开
    if (isOpened_)
    {
        spdlog::error("[" + param_.Key + "] - SerialPort was released failure because it has been opened");
        spdlog::info(LOG_END);
        return false;
    }

    // 判断串口是否初始化完毕
    if (!isInitialized_)
    {
        spdlog::warn("[" + param_.Key + "] - SerialPort can not be released repeatly");
        spdlog::info(LOG_END);
        return false;
    }

    // 删除读取缓冲区
    if (readBuffer_ != nullptr)
    {
        delete[] readBuffer_;
        readBuffer_ = nullptr;
        bytesToRead_ = 0;
    }

    // 删除写入缓冲区
    if (writeBuffer_ != nullptr)
    {
        delete[] writeBuffer_;
        writeBuffer_ = nullptr;
        bytesToWrite_ = 0;
    }

    // 重置串口的初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    spdlog::info("[" + param_.Key + "] - SerialPort was released successful");
    spdlog::info(LOG_END);

    // 返回清理结果
    return true;
}

// 打开串口
bool SerialPort::Open()
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);


    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经初始化
    if (!isInitialized_)
    {
        spdlog::error("[" + param_.Key + "] - SerialPort was opened failure because it has not been initialized");
        spdlog::info(LOG_END);
        return false;
    }

    // 判断串口是否已经打开
    if (isOpened_)
    {
        spdlog::warn("[" + param_.Key + "] - SerialPort can not be opened repeatedly");
        spdlog::info(LOG_END);
        return false;
    }

    // 打开串口
    // O_RDWR：以读写的方式打开串口
    // O_NOCTTY：调用程序不会成为串口的控制终端；如果没有设置该标志，任何一个输入(例如键盘中止信号等)都将影响串口工作
    // O_NONBLOCK：以非阻塞模式打开串口
    fileDescriptor_ = ::open(param_.Name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fileDescriptor_ == -1)
    {
        spdlog::error("[" + param_.Key + "] - SerialPort's fileDescriptor was got failure");
        spdlog::info(LOG_END);
        return false;
    }
    else
    {
        spdlog::info("[" + param_.Key + "] - SerialPort's fileDescriptor was got successful");
    }

    // 读取串口配置信息
    // 参考资料：https://man7.org/linux/man-pages/man3/termios.3.html
    //         https://www.cnblogs.com/senior-engineer/p/10588253.html
    termios options;
    if (::tcgetattr(fileDescriptor_, &options) == -1)
    {
        // 关闭串口
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;

        // 记录日志信息，返回串口打开结果
        spdlog::error("[" + param_.Key + "] - SerialPort's termios was got failure");
        spdlog::info(LOG_END);
        return false;
    }
    else
    {
        spdlog::info("[" + param_.Key + "] - SerialPort's termios was got successful");
    }

    // 修改串口的输入波特率
    if (::cfsetispeed(&options, static_cast<unsigned int>(param_.BaudRate)) == -1)
    {
        // 关闭串口
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;

        // 记录日志信息，返回串口打开结果
        spdlog::error("[" + param_.Key + "] - SerialPort's Input BaudRate was set failure");
        spdlog::info(LOG_END);
        return false;
    }
    else
    {
        spdlog::info("[" + param_.Key + "] - SerialPort's Input BaudRate was set successful");
    }

    // 修改串口的输出波特率
    if (::cfsetospeed(&options, static_cast<unsigned int>(param_.BaudRate)) == -1)
    {
        // 关闭串口
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;

        // 记录日志信息，返回串口打开结果
        spdlog::error("[" + param_.Key + "] - SerialPort's Output BaudRate was set failure");
        spdlog::info(LOG_END);
        return false;
    }
    else
    {
        spdlog::info("[" + param_.Key + "] - SerialPort's Output BaudRate was set successful");
    }

    // c_iflag flag constants:
    // IGNBRK - Ignore BREAK condition on input.
    // BRKINT - If IGNBRK is set, a BREAK is ignored.  If it is not set but BRKINT is set, then a BREAK causes
    //          the input and output queues to be flushed, and if the terminal is the controlling terminal of
    //          a foreground process group, it will cause a SIGINT to be sent to this foreground process group.
    //          When neither IGNBRK nor BRKINT are set, a BREAK reads as a null byte ('\0'), except when PARMRK
    //          is set, in which case it reads as the sequence \377 \0 \0.
    // IGNPAR - Ignore framing errors and parity errors.
    // PARMRK - If this bit is set, input bytes with parity or framing errors are marked when passed to the program.
    //          This bit is meaningful only when INPCK is set and IGNPAR is not set. The way erroneous bytes are
    //          marked is with two preceding bytes, \377 and \0. Thus, the program actually reads three bytes for
    //          one erroneous byte received from the terminal. If a valid byte has the value \377, and ISTRIP
    //          (see below) is not set, the program might confuse it with the prefix that marks a parity error.
    //          Therefore, a valid byte \377 is passed to the program as two bytes, \377 \377, in this case.    //
    //          If neither IGNPAR nor PARMRK is set, read a character with a parity error or framing error as \0.
    // INPCK - Enable input parity checking.
    // ISTRIP - Strip off eighth bit.
    // INLCR - Translate NL to CR on input.
    // IGNCR - Ignore carriage return on input.
    // ICRNL - Translate carriage return to newline on input (unless IGNCR is set).
    // IUCLC - (not in POSIX) Map uppercase characters to lowercase on input.
    // IXON - Enable XON/XOFF flow control on output.
    // IXANY - (XSI) Typing any character will restart stopped output. (The default is to allow just the START
    //         character to restart output.)
    // IXOFF - Enable XON/XOFF flow control on input.
    // IMAXBEL - (not in POSIX) Ring bell when input queue is full. Linux does not implement this bit, and acts
    //           as if it is always set.
    // IUTF8 - (since Linux 2.6.4, not in POSIX) Input is UTF8; this allows character-erase to be correctly performed
    //         in cooked mode.

    // c_oflag flag constants:
    // OPOST - Enable implementation-defined output processing.
    // OLCUC - (not in POSIX) Map lowercase characters to uppercase on output.
    // ONLCR - (XSI) Map NL to CR-NL on output.
    // OCRNL - Map CR to NL on output.
    // ONOCR - Don't output CR at column 0.
    // ONLRET - Don't output CR.
    // OFILL - Send fill characters for a delay, rather than using a timed delay.
    // OFDEL - Fill character is ASCII DEL (0177).  If unset, fill character is ASCII NUL ('\0').
    //         (Not implemented on Linux.)
    // NLDLY - Newline delay mask. Values are NL0 and NL1. [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
    // NL0 - Newline delay
    // NL1 - Newline delay
    // CRDLY - Carriage return delay mask.  Values are CR0, CR1, CR2, or CR3.  [requires _BSD_SOURCE or _SVID_SOURCE
    //         or _XOPEN_SOURCE]
    // CR0 - Carriage return delay
    // CR1 - Carriage return delay
    // CR2 - Carriage return delay
    // CR3 - Carriage return delay
    // TABDLY - Horizontal tab delay mask. Values are TAB0, TAB1, TAB2, TAB3 (or XTABS, but see the BUGS section).
    //          A value of TAB3, that is, XTABS, expands tabs to spaces (with tab stops every eight columns).
    //          [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
    // TAB0 - Horizontal tab delay
    // TAB1 - Horizontal tab delay
    // TAB2 - Horizontal tab delay
    // TAB3 - Horizontal tab delay
    // XTABS - Horizontal tab delay
    // BSDLY - Backspace delay mask. Values are BS0 or BS1. (Has never been implemented.)
    //        [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
    // BS0 - Backspace delay
    // BS1 - Backspace delay
    // VTDLY - Vertical tab delay mask. Values are VT0 or VT1.
    // VT0 - Vertical tab delay
    // VT1 - Vertical tab delay
    // FFDLY - Form feed delay mask. Values are FF0 or FF1. [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
    // FF0 - Form feed delay
    // FF1 - Form feed delay

    // c_cflag flag constants:
    // CBAUD - (not in POSIX) Baud speed mask (4+1 bits). [requires _BSD_SOURCE or _SVID_SOURCE]
    // CBAUDEX - (not in POSIX) Extra baud speed mask (1 bit), included in CBAUD. [requires _BSD_SOURCE or _SVID_SOURCE]
    //           (POSIX says that the baud speed is stored in the termios structure without specifying where precisely,
    //           and provides cfgetispeed() and cfsetispeed() for getting at it. Some systems use bits selected by
    //           CBAUD in c_cflag, other systems use separate fields, for example, sg_ispeed and sg_ospeed.)
    // CSIZE - Character size mask. Values are CS5, CS6, CS7, or CS8.
    // CS5 - 5bit/character
    // CS6 - 6bit/character
    // CS7 - 7bit/character
    // CS8 - 8bit/character
    // CSTOPB - Set two stop bits, rather than one.
    // CREAD - Enable receiver.
    // PARENB - Enable parity generation on output and parity checking for input.
    // PARODD - If set, then parity for input and output is odd; otherwise even parity is used.
    // HUPCL - Lower modem control lines after last process closes the device (hang up).
    // CLOCAL - Ignore modem control lines.
    // LOBLK - (not in POSIX) Block output from a noncurrent shell layer. For use by shl (shell layers).
    //         (Not implemented on Linux.)
    // CIBAUD - (not in POSIX) Mask for input speeds. The values for the CIBAUD bits are the same as the values for
    //          the CBAUD bits, shifted left IBSHIFT bits. [requires _BSD_SOURCE or _SVID_SOURCE] (Not implemented on Linux.)
    // CMSPAR - Use "stick" (mark/space) parity (supported on certain serial devices):
    //          if PARODD is set, the parity bit is always 1;
    //          if PARODD is not set, then the parity bit is always 0.
    // CRTSCTS - (not in POSIX) Enable RTS/CTS (hardware) flow control. [requires _BSD_SOURCE or _SVID_SOURCE]

    // c_lflag flag constants:
    // ISIG - When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal.
    // ICANON - Enable canonical mode.
    // XCASE - (not in POSIX; not supported under Linux) If ICANON is also set, terminal is uppercase only. Input is
    //         converted to lowercase, except for characters preceded by \. On output, uppercase characters are
    //         preceded by \ and lowercase characters are converted to uppercase.
    //         [requires _BSD_SOURCE or _SVID_SOURCE or _XOPEN_SOURCE]
    // ECHO - Echo input characters.
    // ECHOE - If ICANON is also set, the ERASE character erases the preceding input character, and WERASE erases
    //         the preceding word.
    // ECHOK - If ICANON is also set, the KILL character erases the current line.
    // ECHONL - If ICANON is also set, echo the NL character even if ECHO is not set.
    // ECHOCTL -  (not in POSIX) If ECHO is also set, terminal special characters other than TAB, NL, START, and STOP
    //            are echoed as ^X, where X is the character with ASCII code 0x40 greater than the special character.
    //            For example, character 0x08 (BS) is echoed as ^H. [requires _BSD_SOURCE or _SVID_SOURCE]
    // ECHOPRT - (not in POSIX) If ICANON and ECHO are also set, characters are printed as they are being erased.
    //           [requires _BSD_SOURCE or _SVID_SOURCE]
    // ECHOKE - (not in POSIX) If ICANON is also set, KILL is echoed by erasing each character on the line, as specified
    //          by ECHOE and ECHOPRT. [requires _BSD_SOURCE or _SVID_SOURCE]
    // DEFECHO - (not in POSIX) Echo only when a process is reading. (Not implemented on Linux.)
    // FLUSHO - (not in POSIX; not supported under Linux) Output is being flushed.  This flag is toggled by typing the
    //          DISCARD character. [requires _BSD_SOURCE or _SVID_SOURCE]
    // NOFLSH - Disable flushing the input and output queues when generating signals for the INT, QUIT, and SUSP characters.
    // TOSTOP - Send the SIGTTOU signal to the process group of a background process which tries to write to its
    //          controlling terminal.
    // PENDIN - (not in POSIX; not supported under Linux) All characters in the input queue are reprinted when the next
    //          character is read. (bash(1) handles typeahead this way.) [requires _BSD_SOURCE or _SVID_SOURCE]
    // IEXTEN - Enable implementation-defined input processing. This flag, as well as ICANON must be enabled for the
    //          special characters EOL2, LNEXT, REPRINT, WERASE to be interpreted, and for the IUCLC flag to be effective.

    // The c_cc array defines the terminal special characters. The symbolic indices (initial values) and meaning are:
    // VDISCARD - (not in POSIX; not supported under Linux; 017, SI, Ctrl-O)
    //            Toggle: start/stop discarding pending output. Recognized when IEXTEN is set, and then not passed as input.
    // VDSUSP - (not in POSIX; not supported under Linux; 031, EM, Ctrl-Y)
    //          Delayed suspend character (DSUSP): send SIGTSTP signal when the character is read by the user program.
    //          Recognized when IEXTEN and ISIG are set, and the system supports job control, and then not passed as input.
    // VEOF - (004, EOT, Ctrl-D) End-of-file character (EOF). More precisely: this character causes the pending tty buffer
    //        to be sent to the waiting user program without waiting for end-of-line. If it is the first character of
    //        the line, the read(2) in the user program returns 0, which signifies end-of-file. Recognized when ICANON
    //        is set, and then not passed as input.
    // VEOL - (0, NUL) Additional end-of-line character (EOL). Recognized when ICANON is set.
    // VEOL2 - (not in POSIX; 0, NUL) Yet another end-of-line character(EOL2). Recognized when ICANON is set.
    // VERASE - (0177, DEL, rubout, or 010, BS, Ctrl-H, or also #) Erase character (ERASE). This erases the previous
    //          not-yet-erased character, but does not erase past EOF or beginning-of-line.  Recognized when ICANON is
    //          set, and then not passed as input.
    // VINTR - (003, ETX, Ctrl-C, or also 0177, DEL, rubout) Interrupt character (INTR). Send a SIGINT signal.
    //         Recognized when ISIG is set, and then not passed as input.
    // VKILL - (025, NAK, Ctrl-U, or Ctrl-X, or also @) Kill character (KILL). This erases the input since the last EOF
    //         or beginning-of-line.  Recognized when ICANON is set, and then not passed as input.
    // VLNEXT - (not in POSIX; 026, SYN, Ctrl-V) Literal next (LNEXT). Quotes the next input character, depriving it of
    //          a possible special meaning. Recognized when IEXTEN is set, and then not passed as input.
    // VMIN - Minimum number of characters for noncanonical read (MIN).
    // VQUIT - (034, FS, Ctrl-\) Quit character (QUIT). Send SIGQUIT signal. Recognized when ISIG is set, and then
    //         not passed as input.
    // VREPRINT - (not in POSIX; 022, DC2, Ctrl-R) Reprint unread characters (REPRINT). Recognized when ICANON and
    //            IEXTEN are set, and then not passed as input.
    // VSTART - (021, DC1, Ctrl-Q) Open character (START). Restarts output stopped by the Close character. Recognized
    //          when IXON is set, and then not passed as input.
    // VSTATUS - (not in POSIX; not supported under Linux; status request: 024, DC4, Ctrl-T). Status character (STATUS).
    //           Display status information at terminal, including state of foreground process and amount of CPU time
    //           it has consumed. Also sends a SIGINFO signal (not supported on Linux) to the foreground process group.
    // VSTOP - (023, DC3, Ctrl-S) Stop character (STOP). Close output until Open character typed. Recognized when IXON
    //         is set, and then not passed as input.
    // VSUSP - (032, SUB, Ctrl-Z) Suspend character (SUSP). Send SIGTSTP signal. Recognized when ISIG is set, and then
    //        not passed as input.
    // VSWTCH - (not in POSIX; not supported under Linux; 0, NUL) Switch character (SWTCH). Used in System V to switch
    //          shells in shell layers, a predecessor to shell job control.
    // VTIME - Timeout in deciseconds for noncanonical read (TIME).
    // VWERASE - (not in POSIX; 027, ETB, Ctrl-W) Word erase (WERASE). Recognized when ICANON and IEXTEN are set, and
    //           then not passed as input.

    // 修改串口的输入模式基本配置信息
    options.c_iflag &= ~IGNBRK;
    options.c_iflag &= ~BRKINT;
    options.c_iflag &= ~IGNPAR;
    options.c_iflag &= ~PARMRK;
    options.c_iflag |= INPCK;
    options.c_iflag &= ~ISTRIP;
    options.c_iflag &= ~INLCR;
    options.c_iflag &= ~IGNCR;
    options.c_iflag &= ~ICRNL;
    options.c_iflag &= ~IUCLC;

    // 修改串口的输出模式基本配置信息
    options.c_oflag &= ~OPOST;

    // 修改串口的控制模式基本配置信息
    options.c_cflag |= CREAD;
    options.c_cflag |= CLOCAL;

    // 修改串口的本地模式基本配置信息
    options.c_lflag &= ~ISIG;
    options.c_lflag &= ~ICANON;
    options.c_lflag &= ~ECHO;
    options.c_lflag &= ~ECHOE;
    options.c_lflag &= ~ECHOK;
    options.c_lflag &= ~ECHONL;
    options.c_lflag &= ~IEXTEN;

    // 设置数据长度
    options.c_cflag &= ~CSIZE;
    switch (param_.DataBits)
    {
        case EDataBits::Five:
            options.c_cflag |= CS5;
            break;

        case EDataBits::Six:
            options.c_cflag |= CS6;
            break;

        case EDataBits::Seven:
            options.c_cflag |= CS7;
            break;

        case EDataBits::Eight:
            options.c_cflag |= CS8;
            break;

        default:
            break;
    }

    // 设置停止位
    switch (param_.StopBits)
    {
        case EStopBits::One:
            options.c_cflag &= ~(CSTOPB);
            break;

        case EStopBits::Two:
            options.c_cflag |=  CSTOPB;
            break;;

        default:
            break;
    }

    // 设置校验位
    switch (param_.Parity)
    {
        case EParity::None:
            options.c_cflag &= ~PARENB;
            break;

        case EParity::Odd:
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;

        case EParity::Even:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;

        case EParity::Mark:
            options.c_cflag |= PARENB;
            options.c_cflag |= CMSPAR;
            options.c_cflag |= PARODD;
            break;

        case EParity::Space:
            options.c_cflag |= PARENB;
            options.c_cflag |= CMSPAR;
            options.c_cflag &= ~PARODD;
            break;

        default:
            break;
    }

    // 设置流控
    switch (param_.FlowControl)
    {
        case EFlowControl::None:
            options.c_iflag &= ~IXON;
            options.c_iflag &= ~IXOFF;
            options.c_iflag &= ~IXANY;
            options.c_cflag &= ~CRTSCTS;
            break;

        case EFlowControl::Software:
            options.c_iflag |= IXON;
            options.c_iflag |= IXOFF;
            options.c_iflag |= IXANY;
            options.c_cflag &= ~CRTSCTS;
            break;

        case EFlowControl::Hardware:
            options.c_iflag &= ~IXON;
            options.c_iflag &= ~IXOFF;
            options.c_iflag &= ~IXANY;
            options.c_cflag |= CRTSCTS;
            break;

        default:
            break;
    }

    // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    // 设置串口配置信息
    // TCSANOW: the change occurs immediately.
    // TCSADRAIN: the change occurs after all output written to fd has been transmitted. \n
    //            This option should be used when changing parameters that affect output.
    // TCSAFLUSH: the change occurs after all output written to the object referred by fd has been transmitted, \n
    //            and all input that has been received but not read will be discarded before the change is made.
    if (::tcsetattr(fileDescriptor_, TCSANOW, &options) == -1)
    {
        // 关闭串口
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;

        // 记录日志信息，返回串口打开结果
        spdlog::error("[" + param_.Key + "] - SerialPort's termios was set failure");
        spdlog::info(LOG_END);
        return false;
    }
    else
    {
        spdlog::info("[" + param_.Key + "] - SerialPort's termios was set successful");
    }

    // 启动读取线程
    readSwitch_ = true;
    readFromDriverThread_ = std::thread(&SerialPort::ReadFromDriver, this);
    spdlog::info("[" + param_.Key + "] - SerialPort's ReadFromDriverThread was started successful");
    // 启动写入线程
    writeSwitch_ = true;
    writeToDriverThread_ = std::thread(&SerialPort::WriteToDriver, this);
    spdlog::info("[" + param_.Key + "] - SerialPort's WriteToDriverThread was started successful");

    // 设置串口的打开时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    openTimestamp_ = now.time_since_epoch().count();

    // 设置串口打开状态
    isOpened_ = true;

    // 记录日志信息
    spdlog::info("[" + param_.Key + "] - SerialPort was opened successful");
    spdlog::info(LOG_END);

    // 返回串口打开结果
    return true;
}

// 关闭串口
bool SerialPort::Close()
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);


    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经初始化
    if (!isInitialized_)
    {
        spdlog::error("[" + param_.Key + "] - SerialPort was closed failure because it has not been initialized");
        spdlog::info(LOG_END);
        return false;
    }

    // 判断串口是否已经打开
    if (!isOpened_)
    {
        spdlog::warn("[" + param_.Key + "] - SerialPort can not be closed repeatedly");
        spdlog::info(LOG_END);
        return false;
    }
    // 停止读取线程
    readSwitch_ = false;
    if (readFromDriverThread_.joinable())
    {
        readFromDriverThread_.join();
    }
    spdlog::info("[" + param_.Key + "] - SerialPort's ReadFromDriverThread was stoped successful");

    // 停止写入线程
    writeSwitch_ = false;
    writeConditionVariable_.notify_all();
    if (writeToDriverThread_.joinable())
    {
        writeToDriverThread_.join();
    }
    spdlog::info("[" + param_.Key + "] - SerialPort's WriteToDriverThread was stoped successful");

    // 关闭串口
    if (fileDescriptor_ != -1)
    {
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;
    }

    // 重置串口的打开时间戳
    openTimestamp_ = 0;

    // 重置串口打开状态
    isOpened_ = false;

    // 记录日志信息
    spdlog::info("[" + param_.Key + "] - SerialPort was closed successful");
    spdlog::info(LOG_END);

    // 返回串口关闭结果
    return true;
}

// 获取串口缓冲区中可读取的数据长度(以字节为单位)
unsigned int SerialPort::GetBytesToRead()
{
    std::lock_guard<std::mutex> lockGuard(readMutex_);
    return bytesToRead_;
}

// 获取串口缓冲区中已写入的数据长度(以字节为单位)
unsigned int SerialPort::GetBytesToWrite()
{
    std::lock_guard<std::mutex> lockGuard(writeMutex_);
    return bytesToWrite_;
}

// 从串口缓冲区中读取数据
unsigned int SerialPort::Read(const unsigned int &size, unsigned char *data)
{
    // 判断串口是否已经初始化
    if (!isInitialized_)
    {
        return 0;
    }

    // 锁定读取缓冲区
    std::lock_guard<std::mutex> lockGuard(readMutex_);

    // 初始化实际读取的数据长度
    unsigned int count = 0;

    // 读取数据
    if (bytesToRead_ > size)
    {
        ::memcpy(data, readBuffer_, size);
        ::memcpy(readBuffer_, (readBuffer_ + size), bytesToRead_ - size);
        ::memset((readBuffer_ + bytesToRead_ - size), 0, size);
        count = size;
        bytesToRead_ -= size;
    }
    else
    {
        ::memcpy(data, readBuffer_, bytesToRead_);
        ::memset(readBuffer_, 0, bytesToRead_);
        count = bytesToRead_;
        bytesToRead_ = 0;
    }

    // 返回实际读取的数据长度
    return count;
}

// 根据串口配置参数给定的分隔符数组，从串口缓冲区中读取一行数据
unsigned int SerialPort::ReadLine(unsigned char *data)
{
    // 判断串口是否已经初始化
    if (!isInitialized_)
    {
        return 0;
    }

    // 锁定读取缓冲区
    std::lock_guard<std::mutex> lockGuard(readMutex_);

    // 初始化实际读取的数据长度
    unsigned int count = 0;

    // 判断串口配置参数的分隔符数组是否为空
    if (param_.Separators.empty())
    {
        ::memcpy(data, readBuffer_, bytesToRead_);
        ::memset(readBuffer_, 0, bytesToRead_);
        count = bytesToRead_;
        bytesToRead_ = 0;
    }
    else
    {
        // 根据分隔符数组搜索第一行数据
        unsigned int separatorsSize = param_.Separators.size();
        if (bytesToRead_ >= separatorsSize)
        {
            // 搜索数据缓冲区中第一个匹配的分隔符位置
            int separatorsIndex = -1;
            for (int i = 0; i < (bytesToRead_ - separatorsSize + 1); ++i)
            {
                // 判断在数据缓冲区当前位置是否能够成功匹配分隔符
                bool isMatched = true;
                for (int j = 0; j < separatorsSize; j++)
                {
                    if (*(readBuffer_ + i + j) != param_.Separators[j])
                    {
                        isMatched = false;
                        break;
                    }
                }

                // 如果找到匹配的分隔符位置，则停止搜索
                if (isMatched)
                {
                    separatorsIndex = i;
                    break;
                }
            }

            // 复制匹配到的第一行数据
            if (separatorsIndex >= 0)
            {
                count = separatorsIndex + separatorsSize;
                ::memcpy(data, readBuffer_, count);
                ::memcpy(readBuffer_, (readBuffer_ + count), (bytesToRead_ - count));
                ::memset((readBuffer_ + bytesToRead_ - count), 0, count);
                bytesToRead_ -= count;
            }
        }
    }

    // 返回实际读取的数据长度
    return count;
}

// 向串口缓冲区中写入数据
void SerialPort::Write(const unsigned int &size, const unsigned char *data)
{
    // 判断串口是否已经初始化
    if (!isInitialized_)
    {
        return;
    }

    // 锁定写入缓冲区
    std::lock_guard<std::mutex> lockGuard(writeMutex_);

    // 写入数据
    if ((bytesToWrite_ + size) <= param_.WriteBufferSize)
    {
        ::memcpy((writeBuffer_ + bytesToWrite_), data, size);
        bytesToWrite_ += size;
    }
    else
    {
        if (size >= param_.WriteBufferSize)
        {
            ::memcpy(writeBuffer_, (data + size - param_.WriteBufferSize), param_.WriteBufferSize);
        }
        else
        {
            ::memcpy(writeBuffer_,
                     (writeBuffer_ + bytesToWrite_ + size - param_.WriteBufferSize),
                     (param_.WriteBufferSize - size));
            ::memcpy((writeBuffer_ + param_.WriteBufferSize - size), data, size);
        }

        // 更新写入数据长度
        bytesToWrite_ = param_.WriteBufferSize;
    }

    // 通过条件变量启动写操作
    // 参考网址：https://www.cnblogs.com/haippy/p/3252041.html
    writeConditionVariable_.notify_all();
}

// 注册串口数据接收回调函数
bool SerialPort::RegistDataReceivedHandler(DataReceivedHandler handler, void *userData)
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);


    // 记录日志信息
   spdlog::info(LOG_BEGIN);

    // 判断串口是否已经打开
    if (isOpened_)
    {
//        log = "[" + param_.Key + "] - DataReceivedHandler was registed failure because SerialPort has been opened";
//        logger.Save(ELogType::Error, log);
//        log = LOG_END;
//        logger.Save(ELogType::Info, log);
        spdlog::error("[" + param_.Key + "] - DataReceivedHandler was registed failure because SerialPort has been opened");
        spdlog::info(LOG_END);
        return false;
    }

    // 注册回调函数
    dataReceivedHandler_ = handler;
    dataReceivedUserData_ = userData;

    // 记录日志信息
//    log = "[" + param_.Key + "] - DataReceivedHandler was registed successful";
//    logger.Save(ELogType::Info, log);
//    log = LOG_END;
//    logger.Save(ELogType::Info, log);
    spdlog::error("[" + param_.Key + "] - DataReceivedHandler was registed successful");
    spdlog::info(LOG_END);
    // 返回注册结果
    return true;
}

// 解除注册串口数据接收回调函数
bool SerialPort::UnregistDataReceivedHandler()
{
    // 锁定串口操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);


    // 记录日志信息
    spdlog::info(LOG_BEGIN);

    // 判断串口是否已经打开
    if (isOpened_)
    {
//        log = "[" + param_.Key + "] - DataReceivedHandler was unregisted failure because SerialPort has been opened";
//        logger.Save(ELogType::Error, log);
//        log = LOG_END;
//        logger.Save(ELogType::Info, log);
        spdlog::error("[" + param_.Key + "] - DataReceivedHandler was unregisted failure because SerialPort has been opened");
        spdlog::info(LOG_END);
        return false;
    }

    // 解除注册回调函数
    dataReceivedHandler_ = nullptr;
    dataReceivedUserData_ = nullptr;

    // 记录日志信息
//    log = "[" + param_.Key + "] - DataReceivedHandler was unregisted successful";
//    logger.Save(ELogType::Info, log);
//    log = LOG_END;
//    logger.Save(ELogType::Info, log);
    spdlog::error("[" + param_.Key + "] - DataReceivedHandler was unregisted successful");
    spdlog::info(LOG_END);

    // 返回解除注册结果
    return true;
}

// ******************************  SerialPort类的私有方法  ******************************

// 从串口驱动程序中读取数据
void SerialPort::ReadFromDriver()
{
    // 设置超时时间为1秒
    timespec timeout;
    timeout.tv_sec = 1;
    timeout.tv_nsec = 0;

    // 循环扫描串口驱动程序并读取数据
    while (readSwitch_)
    {
        // 等待串口驱动程序有数据可以读取
        // 参考网址：https://blog.csdn.net/u012252959/article/details/48574711
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fileDescriptor_, &readfds);
        if (::pselect(fileDescriptor_ + 1,
                      &readfds,
                      nullptr,
                      nullptr,
                      &timeout,
                      nullptr) <= 0)
        {
            continue;
        }

        // 从串口中读取数据
        // 参考网址：https://linux.die.net/man/4/tty_ioctl
        unsigned int bytesCount = 0;
        if ((::ioctl(fileDescriptor_, TIOCINQ, &bytesCount) == 0) && (bytesCount > 0))
        {
            // 锁定读取缓冲区
            readMutex_.lock();

            // 读取串口数据
            if ((bytesToRead_ + bytesCount) <= param_.ReadBufferSize)
            {
                ::read(fileDescriptor_, (readBuffer_ + bytesToRead_), bytesCount);
                bytesToRead_ += bytesCount;
            }
            else
            {
                if (bytesCount >= param_.ReadBufferSize)
                {
                    unsigned char buffer[bytesCount];
                    ::read(fileDescriptor_, buffer, bytesCount);
                    ::memcpy(readBuffer_,
                             (buffer + bytesCount - param_.ReadBufferSize),
                             param_.ReadBufferSize);
                }
                else
                {
                    ::memcpy(readBuffer_,
                             (readBuffer_ + bytesToRead_ + bytesCount - param_.ReadBufferSize),
                             (param_.ReadBufferSize - bytesCount));
                    ::read(fileDescriptor_,
                           (readBuffer_ + param_.ReadBufferSize - bytesCount),
                           bytesCount);
                }

                // 更新读取数据长度
                bytesToRead_ = param_.ReadBufferSize;
            }

            // 解锁读取缓冲区
            unsigned int bytesToRead = bytesToRead_;
            readMutex_.unlock();

            // 调用数据接收回调函数
            if (dataReceivedHandler_ != nullptr)
            {
                dataReceivedHandler_(bytesToRead, dataReceivedUserData_);
            }
        }
    }
}

// 向串口驱动程序中写入数据
void SerialPort::WriteToDriver()
{
    while (writeSwitch_)
    {
        // 锁定写入缓冲区
        std::unique_lock<std::mutex> uniqueLock(writeMutex_);

        // 将缓冲区的数据写入串口驱动程序
        if (bytesToWrite_ > 0)
        {
            unsigned int bytesCount = ::write(fileDescriptor_, writeBuffer_, bytesToWrite_);
            if ((bytesToWrite_ >= bytesCount) && (bytesCount > 0))
            {
                ::memcpy(writeBuffer_, (writeBuffer_ + bytesCount), (bytesToWrite_ - bytesCount));
                ::memset((writeBuffer_ + bytesToWrite_ - bytesCount), 0, bytesCount);
                bytesToWrite_ -= bytesCount;
            }
        }
        else
        {
            // 如果写入缓冲区中已经没有数据，阻塞写入线程，减小系统资源消耗
            // 参考网址：https://www.cnblogs.com/haippy/p/3252041.html
            writeConditionVariable_.wait(uniqueLock);
        }
    }
}