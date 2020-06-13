import random
import math
import sys
from time import *



from reinforcementLearner import *


if __name__ == '__main__':
    cart = ReinforcementLearner()
    p, oldp, rhat, r = 0, 0, 0, 0
    state, i, y, steps, failures, failed, startSim = 0, 0, 0, 0, 0, False, True
    
    while steps < cart.max_steps and failures < cart.max_failures:
        if startSim == True:
            print("start the robot")
            now = int(time())
            cart.read_variables()
            state = cart.get_state()
            startSim = False
        action = (random.random()/((2**31) - 1) < (1.0 / (1.0 + math.exp(-max(-50, min(cart.action_weights[state], 50))))))
        #update traces
        cart.action_weights_elig[state] += (1 - cart.lambda_w) * (y - 0.5)
        cart.critic_weights_elig[state] += (1 - cart.lambda_v)
        oldp = cart.critic_weights[state]     # remember prediction for the current state
        cart.do_action(action)                # do action
        cart.read_variables()                 # read new values TODO maybe a bit to close after doing action?!
        state = cart.get_state()              # get new x, dx, t, dt
        
        
        if state < 0:
            failed = True
            failures += 1
            print "Trial " + str(failures) + " was " + str(steps) + " steps or " + str(int(time()) - now) + " seconds"
            steps = 0
            print("restart simulation and get initial start values")
            sleep(0.5)
            state = cart.get_state()
            cart.read_variables()
            r = -1  # reward = -1
            p = 0   # prediction of failure
            startSim = True
        else:
            failed = False
            r = 0   # reward = 0
            p = cart.critic_weights[state]
            
        rhat = r + cart.gamma * p - oldp
        cart.update_all_weights(rhat, failed)
        steps += 1
        
        
        if failures == cart.max_failures:
            print("Failed to balance")
        else:
            print("Pole balanced")
        


 #   while(1):
  #      print(cart.read_variables())
        
    

    
#print(cart.read_variables())
#print(cart.read_variables())