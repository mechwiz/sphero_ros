import numpy as np
'''
Custom RK4 integrator for the adjoint
'''
def rk4(f, x0, t, dt, *args):
    '''
    rk4 integrator scheme
    '''
    for i in t:
        if i == t[0]:
            x = np.array(x0)
        else:
            x = np.vstack((x, x0))
        k1 = f(x0, i, *args)*dt
        k2 = f(x0 + k1/2, i+dt/2, *args)*dt
        k3 = f(x0 + k2/2, i+dt/2, *args)*dt
        k4 = f(x0+k3, i+dt, *args)*dt
        x0 = x0 + (k1 + 2*(k2 + k3) + k4)/6
    return x

def euler(f, x0, t, dt, *args):
    '''
    Euler integrator scheme
    '''
    for i in t:
        if i == t[0]:
            x = np.array(x0)
        else:
            x = np.vstack((x, x0))
        x0 = x0 + f(x0, i, *args)*dt # tadah??
    return x

def monte_carlo(f, xlim, n=200, *args):
    '''
    monte carlo integration scheme to speed up dumb things
    '''
    xsamp = [np.random.uniform(low=i[0], high=i[1], size=n) for i in xlim]
    return np.sum(f(*xsamp))/float(n)
