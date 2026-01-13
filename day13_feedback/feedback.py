# Day 13: Feedback (minimal)

x_ref = 1.0      # desired value
x = 0.2          # current output
K = 2.0          # feedback gain

error = x_ref - x
u = K * error

print("error:", error)
print("control input u:", u)