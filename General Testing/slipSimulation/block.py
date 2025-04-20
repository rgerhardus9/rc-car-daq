from graphviz import Digraph

dot = Digraph()

dot.attr(rankdir='LR', size='10')

dot.node('T', 'Target Speed\n(19 m/s)')
dot.node('C', 'P Controller')
dot.node('S', 'Throttle\nSmoother')
dot.node('SD', 'Slip\nDetection')
dot.node('D', 'Throttle\nDelay Buffer')
dot.node('VM', 'Vehicle Model\n(1st Order)')
dot.node('A', 'Acceleration + Noise\n(IMU)')
dot.node('KF', 'Kalman Filter')
dot.node('WS', 'Wheel Speed Sensor')
dot.node('GPS', 'GPS Sensor')

# Connections
dot.edge('T', 'C')
dot.edge('C', 'S')
dot.edge('S', 'SD')
dot.edge('SD', 'D')
dot.edge('D', 'VM')
dot.edge('VM', 'A')
dot.edge('A', 'KF')
dot.edge('VM', 'WS')
dot.edge('KF', 'SD', label="Est. Speed")
dot.edge('WS', 'SD', label="Wheel Speed")
dot.edge('VM', 'GPS')
dot.edge('GPS', 'KF')

# View in notebook or render
dot.render('control_system_diagram', format='png', view=True)
