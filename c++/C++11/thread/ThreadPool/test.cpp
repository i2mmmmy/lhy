#include <iostream>
#include <future>
#include <functional>
#include <thread>
int add(int a, int b){
    return a + b;
}

std::packaged_task<int(int, int)> ptask(add);
std::future<int> res = ptask.get_future();


int main(){
    // std::thread t(std::move(ptask), 12, 2);
    // std::cout << "fu: " << res.get() << std::endl;

    std::thread t(add, 12, 13);

    t.join();
    std::cout << "fu: " << res.get() << std::endl;
    return 0;
}