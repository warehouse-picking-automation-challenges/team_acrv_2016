Install CUDA
* Instructions not provided

Get cuDNN from NVIDIA
1. Unpack
1. `sudo cp include/* /usr/local/cuda-7.5/include/`
1. `sudo cp lib64/* /usr/local/cuda-7.5/lib64/`

Caffe Instructions
1. `sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler`
1. `sudo apt-get install --no-install-recommends libboost-all-dev`
1. `sudo apt-get install libatlas-base-dev`
1. `sudo apt-get install the python-dev`
1. `sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev`
1. `sudo apt-get install libboost-all-dev`
1. `git clone https://github.com/BVLC/caffe.git`
1. `cd caffe/python`
1. `for req in $(cat requirements.txt); do pip install $req; done`
1. `cd ..`
1. `cp Makefile.config.example Makefile.config`
1. Enter Makefile.config and uncomment `USE_CUDNN := 1`
1. `make all`  
`make test`  
`make runtest`
1. `make pycaffe`
1. To ~/.bashrc add:  
`export PYTHONPATH=/home/apc/co/caffe/python:$PYTHONPATH`
1. 
