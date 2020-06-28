## Install

### Armadillo C++ library for linear algebra and scientific computing

##### Step1:

Download version 9.900.1 http://arma.sourceforge.net/download.html

##### Step2:

```bash
sudo apt install cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
```

##### Step3:

navigate to the Armadillo folder downloaded

```bash
cd armadillo-9.900.1/
```

```bash
cmake .
```

```bash
make
```

```bash
sudo make install
```

##### Usage

```c++
#include <armadillo>
```

```bash
g++ ..... -larmadillo
```





