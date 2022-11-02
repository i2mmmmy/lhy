#include <iostream>
#include <vector>
#include <chrono>

#include "ThreadPool.h"

std::string reason;
class A{
public:
	A(int num):length(num){};

	int length = 0;
	int* m_a_ = new int[length];
};

int status(int index){
	std::shared_ptr<A> t_ptr = std::make_shared<A>(10);
	while(1){
		std::this_thread::sleep_for(std::chrono::seconds(1));
		printf("id: %d, stamp:%ld\n", index, std::time(nullptr));
		if(reason != ""){
			std::cout << "close thread\n" << reason << std::endl;
			return -1;
		}
	}
	return 0;
}

void close(const std::string& why){
	reason = why;
}
int main()
{
	ThreadPool pool(8);
	std::vector< std::future<int> > results;

	for (int i = 0; i < 2; ++i) {

		results.emplace_back(
			// pool.enqueue([i] {
			// 	std::cout << "hello " << i << std::endl;
			// 	std::this_thread::sleep_for(std::chrono::seconds(2));
			// 	std::cout << "world " << i << std::endl;
			// 	return i * i;
			// 	})
			pool.enqueue(status, i)
		);
	}
	std::this_thread::sleep_for(std::chrono::seconds(5));
	std::cout << "hello\n" << std::endl;

	close("nowhy");
	// for (auto&& result : results)
	// 	std::cout << result.get() << ' ';
	// std::cout << std::endl;

	return 0;
}
