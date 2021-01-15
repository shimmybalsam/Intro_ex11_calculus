#!/usr/bin/env python3

import math

EPSILON = 1e-5
DELTA = 1e-3
SEGMENTS = 100
LOW_LIM = -1
HIGH_LIM = 1
DIVISOR = 2
ENLARGE = 2


def plot_func(graph, f, x0, x1, num_of_segments=SEGMENTS, c='black'):
    """
    plot f between x0 to x1 using num_of_segments straight lines
    to the graph object. the function will be plotted in color c
    """
    x = x0
    z = x + (x1 - x0)/num_of_segments
    for i in range(num_of_segments):
        p1 = (x, f(x))
        p2 = (z, f(z))
        graph.plot_line(p1, p2, c)
        x = p2[0]
        z = x + (x1 - x0)/num_of_segments


def const_function(c):
    """return the mathematical function f such that f(x) = c
    >>> const_function(2)(2)
    2
    >>> const_function(4)(2)
    4
    """
    def f(x): return c
    return f


def identity():
    """return the mathematical function f such that f(x) = x

    >>> identity()(3)
    3
    """
    def f(x): return x
    return f


def sin_function():
    """return the mathematical function f such that f(x) = sin(x)
    >>> sin_function()(math.pi/2)
    1.0
    """
    def f(x): return math.sin(x)
    return f


def sum_functions(g, h):
    """return f s.t. f(x) = g(x)+h(x)"""
    def f(x): return g(x) + h(x)
    return f


def sub_functions(g, h):
    """return f s.t. f(x) = g(x)-h(x)"""
    def f(x): return g(x) - h(x)
    return f


def mul_functions(g, h):
    """return f s.t. f(x) = g(x)*h(x)"""
    def f(x): return g(x)*h(x)
    return f


def div_functions(g, h):
    """return f s.t. f(x) = g(x)/h(x)"""
    def f(x): return g(x)/h(x)
    return f


def solve(f, x0=-1000, x1=1000, epsilon=EPSILON):
    """
    Find a solution to f in the range x0 and x1
    assuming that f is monotnic.
    If no solution was found return None
    """
    if abs(f(x0)) < epsilon:
        return x0
    if abs(f(x1)) < epsilon:
        return x1
    if f(x0) * f(x1) >= 0:
        return None
    left_edge = x0
    right_edge = x1
    if f(x0) > f(x1):
        left_edge, right_edge = x1, x0
    x = (left_edge + right_edge) / DIVISOR
    while abs(f(x)) >= epsilon:
        if f(x) > 0:
            right_edge = x
        else:
            left_edge = x
        x = (left_edge + right_edge) / DIVISOR
    return x


def inverse(g, epsilon=EPSILON):
    """
    return f s.t. f(g(x)) = x
    g must be monotonic and continuous
    """
    def f(y):
        idea = sub_functions(g, const_function(y))
        x0 = LOW_LIM
        x1 = HIGH_LIM
        while idea(x0) * idea(x1) > 0:
            x0 *= ENLARGE
            x1 *= ENLARGE
        return solve(idea,x0,x1,epsilon)
    return f


def compose(g, h):
    """return the f which is the compose of g and h """
    def f(x): return g(h(x))
    return f


def derivative(g, delta=DELTA):
    """return f s.t. f(x) = g'(x)"""
    def f(x): return (g(x + delta) - g(x)) / delta
    return f


def definite_integral(f, x0, x1, num_of_segments=SEGMENTS):
    """
    return a float - the definite_integral of f between x0 and x1
    >>> round(definite_integral(const_function(3),-2,3),6)
    15.0
    """
    NEXT = (x1 - x0)/num_of_segments
    x = x0
    integral = 0
    for i in range(num_of_segments):
        next_x = x + NEXT
        integral += f((x + next_x)/DIVISOR) * NEXT
        x = next_x
    return integral


def integral_function(f, delta=0.01):
    """return F such that F'(x) = f(x)"""
    def F(x):
        num_of_segments = int(math.ceil(math.fabs(x) / delta))
        if x > 0:
            return definite_integral(f,0,x,num_of_segments)
        elif x < 0:
            return definite_integral(f,x,0,num_of_segments)*-1
        else:
            return 0
    return F


def ex11_func_list():
    """return list with the functions in q.13"""
    def f0():
        """return f s.t f(x) = 4"""
        return const_function(4)

    def f1():
        """return f s.t f(x) = 3-sin(x)"""
        return sub_functions(const_function(3),sin_function())

    def f2():
        """return f s.t f(x) = sin(x-2)"""
        return compose(sin_function(),(sub_functions(identity(),
                                                     const_function(2))))

    def f3():
        """return f s.t f(x) = 10/[2+sin(x)+x**2]"""
        inner1 = sum_functions(const_function(2),sin_function())
        inner2 = mul_functions(identity(),identity())
        return div_functions(const_function(10),sum_functions(inner1,inner2))

    def f4():
        """return f s.t f(x) = cos(x)/(sin(x)-2)"""
        cos = derivative(sin_function())
        return div_functions(cos,sub_functions(sin_function(),
                                               const_function(2)))

    def f5():
        """return f s.t f(x) = -0.1*integral(0.3x**2+0.7x-1)"""
        a = mul_functions(const_function(0.3),mul_functions(identity(),
                                                            identity()))
        b = mul_functions(const_function(0.7),identity())
        c = const_function(1)
        integral = integral_function(sub_functions(sum_functions(a,b),c))
        return mul_functions(const_function(-0.1),integral)

    def f6():
        """return f s.t f(x) = [cos(sin(x))-0.3*cos(x)]*2"""
        trig_part = compose(derivative(sin_function()), sin_function())
        extra_part = mul_functions(const_function(0.3),
                                   derivative(sin_function()))
        return mul_functions(const_function(2),
                             sub_functions(trig_part, extra_part))

    def f7():
        """return f s.t f is the inverse function to g(x) = 2-x^3"""
        x_power_3 = mul_functions(mul_functions(identity(),identity()),
                                  identity())
        g = sub_functions(const_function(2),x_power_3)
        return inverse(g)

    func_list = [f0(),f1(),f2(),f3(),f4(),f5(),f6(),f7()]
    return func_list


# func that generates the figure in the ex description
def example_func(x):
    return (x/5)**3

if __name__ == "__main__":
    import tkinter as tk
    from ex11helper import Graph
    # un-tag the following lines to activate the doctests
    # import doctest
    # doctest.testmod()
    master = tk.Tk()
    graph = Graph(master, -10, -10, 10, 10)
    # un-tag the line below after implementation of plot_func
    plot_func(graph,example_func,-10,10,SEGMENTS,'red')
    color_arr = ['black', 'blue', 'red', 'green', 'brown', 'purple',
                  'dodger blue', 'orange']
    # un-tag the lines below after implementation of ex11_func_list
    for f in ex11_func_list():
        plot_func(graph, f, -10, 10, SEGMENTS, color_arr[0])
        color_arr.remove(color_arr[0])
    master.mainloop()
