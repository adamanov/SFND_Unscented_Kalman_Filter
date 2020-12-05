# SFND Unscented Kalman Filter
Sensor Fusion UKF Highway Project

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project I implemented an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project required obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 


## Project Instructions and [Rubric](https://review.udacity.com/#!/rubrics/2551/view)

## Advance Learning Tips
- [Applying the unscented Kalman filter for nonlinear state estimation](https://www.sciencedirect.com/science/article/pii/S0959152407001655)
- [Uncented Kalman Filter for Dummies](https://robotics.stackexchange.com/questions/9233/unscented-kalman-filter-for-dummies)
- [Learning the Unscented Kalman Filter](https://www.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter?w.mathworks.com)
- [Introduction to Unscented Kalman Filter](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/AV0809/qi.pdf)
- [Unscented Kalman Filter Tutorial](https://www.cse.sc.edu/~terejanu/files/tutorialUKF.pdf)
- [KF, EKF and UKF](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam05-ukf.pdf)
- [Application of Unscented Kalman Filter to a cable driven surgical robot](https://ieeexplore.ieee.org/document/6224776)
- [TheUnscentedKalmanFilterforNonlinearEstimatio](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)



## Advance Learning Tips

Below are some resources of strategies and techniques for Debugging Lots of C++ Objects effectively.

- [Debugging Strategies, Tips, and Gotchas](https://www.cprogramming.com/debugging/debugging_strategy.html)
- [Techniques for Debugging in C++](https://accu.org/journals/overload/9/46/goodliffe_423/)
- [Some favorite debugging techniques in C++ as discussed on stack overflow](https://stackoverflow.com/questions/1325853/what-are-your-favorite-debugging-techniques-in-c)

## Code Efficiency 

Here are a few tips for improving on code efficiency and optimization:

- The most efficient types:
    - When defining an object to store an integer number, use the int or the unsigned int type, except when a longer type is needed
    - When defining an object to store a character, use the char type, except when the wchar_t type is needed
    - When defining an object to store a floating point number, use the double type, except when the long double type is needed.
    - If the resulting aggregate object is of medium or large size, replace each integer type with the smallest integer type that is long enough to contain it (but without using [bit-fields](https://en.cppreference.com/w/cpp/language/bit_field)) and replace the floating point types with the [float type](https://www.learncpp.com/cpp-tutorial/floating-point-numbers/), except when greater precision is needed.

- This [article](https://www.thegeekstuff.com/2015/01/c-cpp-code-optimization/) will give some high-level ideas on how to improve the speed of your program. This inlucdes the printf and scanf Vs cout and cin, Using Operators, if Condition Optimization, Problems with Functions, Optimizing Loops, Data Structure Optimization and a lot more.
- [Optimizing C++/Writing efficient code/Performance improving features.](https://en.wikibooks.org/wiki/Optimizing_C%2B%2B/Writing_efficient_code/Performance_improving_features)
- [Efficient C++ Performance Programming Techniques](http://www.whigg.ac.cn/resource/program/CPP/201010/P020101023562491092566.pdf)

## More about Kalman Filters
- [Kalman Filter, Extended Kalman Filter, Unscented Kalman Filter](https://medium.com/@kastsiukavets.alena/kalman-filter-extended-kalman-filter-unscented-kalman-filter-dbbd929f83c5)
- [USDC Extended Kalman Filters — my bits](https://tempflip.medium.com/udacity-self-driving-cars-extended-kalman-filters-my-bits-99cbbaf65e3d)




**SFND_Unscented_Kalman_Filter**

The main project empty folder can be found [here](https://github.com/udacity/SFND_Unscented_Kalman_Filter)
