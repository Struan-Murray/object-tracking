#include <iostream>
#include <chrono>

int main()
{
	auto p = std::chrono::high_resolution_clock::now();
	intmax_t now = 
(uintmax_t)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
	std::cout << now << "\n";;
return 0;
}
