<div align="center">
  	<h1>
    	BdryReach System User Guide
  	</h1>
	<br />
    <div>
		<a href="https://github.com/BdryReach/BdryReach">
			<img width="428" src="free_logo (1).svg" alt="BdryReach System">
		</a>
	</div>
</div>

## 1. Tool Installation
**All experiments are conducted on a virtual machine system with an Intel® Core™ i7-10750H CPU @ 2.60GHz × 8 and 5.8 GiB of memory, running Ubuntu 20.04.3 LTS. We recommend a similar hardware setup and a Linux environment. The installation requires the cmake tool and various third-party libraries needed for BdryReach.**

### 1.1 Required Third-Party Libraries

| Library | Website | Version |
| --- | --- | --- |
| git | [https://git-scm.com/](https://git-scm.com/) | 2.25.1 |
| Cmake | [https://cmake.org/](https://cmake.org/) | latest |
| Eigen | [http://eigen.tuxfamily.org/index.php?title=Main Page](http://eigen.tuxfamily.org/index.php?title=Main%20Page) | 3.34 |
| Python | [https://www.python.org/](https://www.python.org/) | 2.7.18 |
| Capd | [http://capd.ii.uj.edu.pl/](http://capd.ii.uj.edu.pl/) | latest |
| boost | [https://www.boost.org/](https://www.boost.org/) | 1.67.0.0 |
| GLPK | [https://www.gnu.org/software/glpk/](https://www.gnu.org/software/glpk/) | 4.65-2 |

### 1.2 Installation Commands for Some Third-Party Libraries

#### 1.2.1 Eigen3
```bash
sudo apt-get install libeigen3-dev
```
#### 1.2.2 Python
```bash
sudo apt-get install python-dev
```
#### 1.2.3 Capd
```bash
svn co https://svn.capdnet.ii.uj.edu.pl/capd/
sudo apt install libtool
autoreconf --install
mkdir capd_nogui
cd capd_nogui
../capd/configure --prefix=/usr/local --without-gui --without-mpfr
make
sudo make install
```
### 1.3 BdryReach Toolkit Download and Test Case Compilation
```bash
git clone https://gitee.com/ren-dejin/BdryReach.git
cd BdryReach/
mkdir build
cd build
cmake ..
make
```
## 2. Usage

### 2.1 Reachable Set Approximation Interface
### 2.1.1   Reachable Set Upper Approximation Interface
```cpp
template <typename Number>
static vector<ReachableSet<Number>> BdReach(NonlinearSys<Number> mysys, ReachOptions<Number> options, Zonotope<Number> R0)
```
**Parameters:**
* **mysys:** Differential equation for computing reachable sets.
* **options:** Configuration for upper reachable set approximation.
* **R0:** Initial set.


### 2.1.2 Reachable Set Lower Approximation Interface
```cpp
template <typename Number>
        static vector<Zonotope<Number>> underReachClp(NonlinearSys<Number> mysys, 			
        NonlinearSys<Number> mysysBack, ReachOptions<Number> options, Zonotope<Number> R0, double 	
        overRtime, int steps, double radius, double over_step, double bound_step, int Zover_order)
```
**Parameters:**
* **mysys:** Differential equation for computing reachable sets.
* **mysysBack:** Reverse differential equation for result verification.
* **options:** Configuration for the lower reachable set approximation part of the program.
* **R0:** Initial set.
* **overRtime:** Step size for each step of the lower reachable set approximation.
* **steps:** Iteration count for the lower reachable set approximation.
* **radius:** Maximum generator length allowed for boundary segmentation.
* **over_step:** Step size for computing the entire reachable set approximation in each step of the lower reachable set approximation.
* **bound_step:** Step size for computing the boundary reachable set approximation in each step of the lower reachable set approximation.
* **Zover_order:** Limit on the zonotope order for computing the entire reachable set approximation in each step of the lower reachable set approximation.
### 2.2 Reachable Set Upper Approximation Example
**As an example, we calculate the upper reachable set approximation for the VanderPol model. The file computes the reachable set approximation from the initial region ([1.23,1.57],[2.34,2.46]) for the time interval 0-6.74s. The specific file location is:**
```RobotFramework
/examples/overVanderPol.cpp.
```
### 2.2.1 Include Files
```cpp
#include <overApprox/overApprox.h> // Header file containing interfaces for computing reachable set over-approximation
#include <plotter/matplotlibcpp.h> // Header file for Matplotlib C++ plotting library
#include <plotter/plotter.h> // Header file for result plotting
```
### 2.2.2 Define Differential Equation
**We define the form of differential equations using the Capd library. For detailed information on the differential equation system in Capd, [please refer to the Capd documentation on differential equation systems.](https://capd.sourceforge.net/capdDynSys/docs/html/maps.html)**


```cpp
double mu = 1.;

void _f(Node/* t*/, Node in[], int /*dimIn*/, Node out[], int/* dimOut*/, Node params[], int noParams){
    out[0] = in[1];
    out[1] = mu * (1-in[0]*in[0])*in[1] - in[0]/*+ in[2]*/;
}

// Input dimension of the differential equation
int dimIn = 3; // The input dimension of the differential equation. Since the default input includes control u, the input dimension is one greater than the output dimension.

// Output dimension of the differential equation
int dimOut = 2; // The output dimension of the differential equation.

// Parameter settings for the differential equation. Since this differential equation has no parameters, it is set to 0.
int noParam = 0;

// Maximum order for Taylor expansion of the differential equation
int MaxDerivativeOrder = 3; // The maximum order to which the differential equation is expanded using Taylor series.

// Creating IMap for interval computations
IMap f(_f, dimIn, dimOut, noParam, MaxDerivativeOrder); // Constructing IMap for interval calculations


```
### 2.2.3 Set Reachable Set Parameters
**Here, we adopt the parameter definitions similar to the MATLAB Reachable Set Computation Toolbox (CORA). For the specific meanings of each parameter, please refer to the documentation provided by CORA.**
```cpp
    NonlinearSys<double> mysys(f, 2, 0, 2);
    ReachOptions<double> options;

    //create R0
    Vector_t<double> center(2);
    Vector_t<double> c(2);
    c << 1.4, 2.4;
    Matrix_t<double> generators(2,1);
    Matrix_t<double> G(2,2);
    G<< 0.17,0,
                 0,0.06;
    Zonotope<double> R0_(c,G);

    center << 1.4, 2.46;
    generators<< 0.17,
                 0;

    options.set_R0(R0_);

    options.set_time_step(0.005);
    options.set_taylor_terms(4);
    options.set_zonotope_order(50);
    options.set_intermediate_order(50);
    options.set_error_order(20);
    options.set_alg("lin");
    options.set_tensor_order(3);

    options.set_tFinal(6.74);
    options.set_tStart(0);

    options.set_usekrylovError(1);
    options.set_max_error(DBL_MAX*Eigen::MatrixXd::Ones(2,1));
```
### 2.2.4 Call Boundary-Based Upper Reachable Set Approximation Method
**This step involves invoking our boundary-based reachable set approximation method. Please refer to Section 2.1.1 for the meanings of various parameters.**
```cpp
vector<ReachableSet<double>> BdReachset = OverApprox::BdReach(mysys, options, R0_);
```
### 2.2.5 Plot Results
**For drawing the graphical representation of the results, we utilize the lightweight plotting library Matplotlib for C++. For specific usage instructions,[please refer to Matplotlib for C++ Documentation.](https://matplotlib-cpp.readthedocs.io/en/latest/index.html)**
```cpp
plt::figure_size(1200, 780);
for(int i = 0; i < BdReachset.size(); i++){
    Plotter::plotReach(BdReachset[i], 1, 2, "b");
}
plt::show();
```
### 2.2.6 Results Display
**Results are displayed by comparing the upper reachable set approximations obtained using BdryReach and CORA. The blue portion represents the results calculated by BdryReach, while the red portion is calculated by CORA. It is evident that BdryReach achieves higher accuracy in upper reachable set approximation compared to CORA.**
<p align="center">
  <img src=result_picture/2.2.6.png>
</p>

## 2.3 Approximate Computation of Reachable Sets Use Case
**In this section, we illustrate the computation of reachable sets under approximation for the VanderPol model. The file calculates an under-approximation of the reachable set starting from the initial region ([1.23, 1.57], [2.34, 2.46]), with a step size of 0.1s and a time interval of 0-0.8s. The specific file location is /examples/underVanderPol.cpp.**
### 2.3.1 Include Files

```cpp
#include <plotter/matplotlibcpp.h>   // Header for result visualization
#include <plotter/plotter.h>          // Header for result visualization
#include <underApprox/underApprox.h>  // Header for includes the interface for computing reachable sets under approximation.
```
### 2.3.2 Differential Equation Definitions

**We use the Capd library to define the form of the differential equations. Refer to the Capd documentation on [differential equation systems](https://capd.sourceforge.net/capdDynSys/docs/html/maps.html). Additionally, a reverse differential equation is defined for validation purposes.**

```cpp
double mu = 1.;

void _f(Node/* t*/, Node in[], int /*dimIn*/, Node out[], int/* dimOut*/, Node params[], int noParams){
    // Forward differential equation
    out[0] = in[1];
    out[1] = mu * (1-in[0]*in[0])*in[1] - in[0];
}

void _fBack(Node/* t*/, Node in[], int /*dimIn*/, Node out[], int/* dimOut*/, Node params[], int noParams){
    // Reverse differential equation
    out[0] = -in[1];
    out[1] = -(mu * (1-in[0]*in[0])*in[1] - in[0]);
}

int dimIn = 3;
int dimOut = 2;
int noParam = 0;
int MaxDerivativeOrder = 3;

IMap f(_f, dimIn, dimOut, noParam, MaxDerivativeOrder);
IMap fBack(_fBack, dimIn, dimOut, noParam, MaxDerivativeOrder);
```
### 2.3.3 Parameters for Computing Reachable Sets

**We adopt parameter definitions similar to the MATLAB Reachability Analysis Toolbox CORA. For detailed meanings, refer to CORA's documentation.**
```cpp
NonlinearSys<double> mysys(f, 2, 0, 2);
NonlinearSys<double> mysysBack(fBack, 2, 0, 2);

ReachOptions<double> options;

// Create R0
Vector_t<double> center(2);
center << 1.4, 2.4;
Matrix_t<double> generators(2,2);
generators << 0.17, 0,
              0, 0.06;

Zonotope<double> R0_(center, generators);

options.set_taylor_terms(4);
options.set_zonotope_order(50);
options.set_intermediate_order(50);
options.set_error_order(20);
options.set_alg("lin");
options.set_tensor_order(3);

options.set_tFinal(6);
options.set_tStart(0);

options.set_R0(R0_);

options.set_usekrylovError(1);
options.set_max_error(DBL_MAX * Eigen::MatrixXd::Ones(2,1));
```
### 2.3.4 Invocation of Boundary-Based Under-Approximation Calculation Method

**This step invokes our boundary-based under-approximation calculation method. Refer to section 2.1.2 for parameter meanings.**
```cpp
vector<Zonotope<double>> underR = UnderApprox::underReachClp(mysys, mysysBack, options, R0_, 0.1, 8, 0.01, 0.05, 0.01, 50);
```
### 2.3.5 Result Visualization

**For the visualization of results, we use the lightweight plotting library Matplotlib for C++. [Refer to Matplotlib for C++ documentation.](https://matplotlib-cpp.readthedocs.io/en/latest/index.html)**
```cpp
plt::figure_size(1200, 780);
for(int i = 1; i < underR.size(); i++){
    Plotter::plotZonotope(underR[i], 1, 2, "g");
}
Plotter::plotZonotope(R0_, 1, 2, "k");
plt::show();
```
### 2.3.6 Results Presentation

**We compute the under-approximation of the VanderPol model starting from the initial region ([1.23, 1.57], [2.34, 2.46]) with a step size of 0.1s and a time interval of 0-0.8s. The green area represents the under-approximation of the reachable set, while the blue area represents the over-approximation for comparison, showing the precision of the under-approximation calculation.**
<p align="center">
  <img src=result_picture/2.3.6.png>
</p>

## 3 Reproducing Results

**All experiments were conducted on a virtual machine with an Intel® Core™ i7-10750H CPU @ 2.60GHz × 8 and 5.8 GiB of memory, running Ubuntu 20.04.3 LTS. The experiment files are located in the /examples directory. Note that different environments may yield some variations in results. We provide 8 C++ files to reproduce all experiments. To run these experiments, execute the following CMake statements in the /BdaryReach directory.**
```bash
mkdir build
cd build
cmake ..
make

./examples/underBiological
./examples/underTank
./examples/underVanderPol
./examples/underElectroOsc
./examples/overElectroOsc
./examples/overTank
./examples/overVanderPol
./examples/over2Rrobot
```
## 4 Frequently Asked Questions and Troubleshooting
### Slow Execution
* **For reachable set over-approximation, check if a large initial region or time interval is set. Larger initial regions or time intervals lead to longer computation times. It is recommended to reduce the initial region size or decrease the computation time interval.**
* **For reachable set under-approximation, check if a small radius is set (maximum allowed generator length for boundary segmentation). A smaller radius in high-dimensional differential equations can significantly increase the number of boundaries, leading to longer computation times. It is advisable to increase the radius.**
### Unsuccessful Reachable Set Over-Approximation
* **Check if a large initial region or a long time interval is set. Large initial regions or long time intervals may cause the "wrapping effect" in reachable set over-approximation, where the computed over-approximation quickly expands. It is recommended to reduce the initial region size or decrease the computation time interval.**
### Unsuccessful Reachable Set Under-Approximation
* **Check if a small initial region or a long time interval is set. A small initial region may result in an empty under-approximation. Moreover, a long time interval may cause the under-approximation to become smaller over time, eventually leading to an empty set.**
 ## 5 Acknowledgments
* **This tool is inspired by the algorithms for reachable set over-approximation in the MATLAB Reachability Analysis Toolbox CORA. We also utilized methods from the Capd library for interval-based derivatives and interval calculations in the system.**
