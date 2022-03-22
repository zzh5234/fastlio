#include <iostream>     // std::cout
#include <algorithm>    // std::fill

using namespace std;

int main () {
  int array[8] = {1};                       // myvector: 0 0 0 0 0 0 0 0

  cout<<"=======begin======="<<"\n"; 
  for (int i = 0;i< 8;i++)
    cout << ' ' << array[i];
  cout << '\n'<<'\n';

  fill (array,array+4,5);   // myvector: 5 5 5 5 0 0 0 0
  fill (array+3,array+6,8);   // myvector: 5 5 5 8 8 8 0 0

  cout<<"=======after fill======="<<"\n"; 
  for (int i = 0;i< 8;i++)
    cout << ' ' << array[i];
  cout << '\n'<<'\n';
  
  return 0;
}
