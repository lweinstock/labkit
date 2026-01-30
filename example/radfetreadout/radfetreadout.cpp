#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

#include <labkit/comms/tcpipcomm.hh>
#include <labkit/devices/jgu/radfet4port.hh>

using namespace std;
using namespace labkit;

int main(int argc, char** argv)
{
    string ip_addr = "192.168.2.111";
    auto tcp = make_unique<TcpipComm>(ip_addr, RadFET4Port::PORT);
    RadFET4Port r4p(std::move(tcp));

    while (true)
    {
        cout << "UFET = " << r4p.getFETVoltage(0) << " T=" << r4p.getDiodeVoltage(0) << endl;
        usleep(100e3);
    }

    return 0;
}