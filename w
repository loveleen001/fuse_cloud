g++ your_code.cpp -o your_program \
  -I~/Scanner/libfreenect2/include \
  -I~/Scanner/libfreenect2/build/include \
  -L~/Scanner/libfreenect2/build/lib \
  -lfreenect2 \
  -std=c++11
