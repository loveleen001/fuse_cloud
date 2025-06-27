g++ multi_capture.cpp -o multi_capture -std=c++17 \
  -I~/Scanner/libfreenect2/include \
  -I/usr/include/pcl-1.12 -I/usr/include/eigen3 \
  -L~/Scanner/libfreenect2/build/lib -lfreenect2 \
  -lpcl_common -lpcl_io
