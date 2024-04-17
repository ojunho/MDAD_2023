from std_msgs.msg import Float64

msg = Float64(42.0)
# msg.data = 42.0
# print(type(msg), type(msg.data), msg.data)
# <class 'std_msgs.msg._float64.Float64'> <class 'float'> 42.0
print(type(msg))