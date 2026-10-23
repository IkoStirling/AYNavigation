#pragma once
#include <cstdio>
#include <stdexcept>
#include <string>
#define CHECK(...) do {if(!(__VA_ARGS__)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+" " #__VA_ARGS__);}while(false)
#define RUN(test) do {test();++cases;std::printf("PASS %s\n",#test);}while(false)
#define TEST_MAIN(body) int main(){int cases=0;try{body;std::printf("%d cases passed\n",cases);return 0;}catch(const std::exception& e){std::fprintf(stderr,"%s\n",e.what());return 1;}}
