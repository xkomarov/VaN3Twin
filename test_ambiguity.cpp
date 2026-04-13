
#include <iostream>
#include <stdint.h>

template <class T>
class VDPDataItem
{
public:
    VDPDataItem(T data) { std::cout << "Data constructor" << std::endl; }
    VDPDataItem(bool availability, void* dummy = nullptr) { std::cout << "Availability constructor" << std::endl; }
    VDPDataItem() {}
};

int main() {
    VDPDataItem<bool> a(false);
    return 0;
}
