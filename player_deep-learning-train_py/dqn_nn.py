# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import tensorflow as tf
import numpy as np

class NeuralNetwork:
	def __init__(self, NetworkToCopy = None, RestoreFromFile = False, ReplayNetwork = False):
		self.frame_res = 5     # Resolution of the input
		self.nframes = 1       # Number of frames/channels of the input
		NumberOfActions = 11    # Number of possible actions
		learning_rate = 1e-4   # Learning Rate		
		
		# Placeholders for the input variables
		self.sess = tf.InteractiveSession()
		self.phy = tf.placeholder(tf.float32, shape=[None, self.frame_res*self.nframes]) # Flattened last nf frames of the game
		self.y = tf.placeholder(tf.float32, shape=[None, 1]) 
		self.action = tf.placeholder(tf.float32, shape=[None, NumberOfActions])

		# Fully-connected layer 1
		input_f1 = self.phy
		fc1_num_neurons = 256
		self.W_fc1 = tf.Variable(tf.truncated_normal([self.frame_res*self.nframes, fc1_num_neurons], stddev=0.01), name="W_fc1")
		self.b_fc1 = tf.Variable(tf.constant(0.01, shape=[fc1_num_neurons]), name="b_fc1")
		self.input_fc1_flat = tf.reshape(input_f1, [-1, self.frame_res*self.nframes])
		self.out_fc1 = tf.nn.relu(tf.matmul(self.input_fc1_flat, self.W_fc1) + self.b_fc1)
        
		# Fully-connected layer 2
		fc2_num_neurons = 256
		self.W_fc2 = tf.Variable(tf.truncated_normal([fc1_num_neurons, fc2_num_neurons], stddev=0.01), name="W_fc2")
		self.b_fc2 = tf.Variable(tf.constant(0.01, shape=[fc2_num_neurons]), name="b_fc2")
		self.out_fc2 = tf.nn.relu(tf.matmul(self.out_fc1, self.W_fc2) + self.b_fc2)

		# Output layer
		out_layer1_input = self.out_fc2
		out_layer1_n_outputs = NumberOfActions
		self.W_ol1 = tf.Variable(tf.truncated_normal([fc2_num_neurons, out_layer1_n_outputs], stddev=0.01), name="W_ol1")
		self.b_ol1 = tf.Variable(tf.constant(0.01, shape=[out_layer1_n_outputs]), name="b_ol1")
		self.Q_theta = tf.matmul(out_layer1_input, self.W_ol1) + self.b_ol1
		self.Q_action = tf.reduce_sum(tf.multiply(self.Q_theta, self.action),  reduction_indices=1, keep_dims=True)
		self.loss_fct = tf.reduce_mean( tf.square(self.y - self.Q_action) )
		self.train_step = tf.train.AdamOptimizer(learning_rate).minimize(self.loss_fct)

		if ReplayNetwork == True:
			self.update_ph_wfc1 = tf.placeholder(self.W_fc1.dtype, shape=[self.frame_res*self.nframes, fc1_num_neurons])
			self.update_ph_wfc2 = tf.placeholder(self.W_fc2.dtype, shape=[fc1_num_neurons, fc2_num_neurons])
			self.update_ph_bfc1 = tf.placeholder(self.b_fc1.dtype, shape=[fc1_num_neurons])
			self.update_ph_bfc2 = tf.placeholder(self.b_fc2.dtype, shape=[fc2_num_neurons])
			self.update_ph_wol1 = tf.placeholder(self.W_ol1.dtype, shape=[fc2_num_neurons, out_layer1_n_outputs])
			self.update_ph_bol1 = tf.placeholder(self.b_ol1.dtype, shape=[out_layer1_n_outputs])

			self.update_op_wfc1 = self.W_fc1.assign(self.update_ph_wfc1)            
			self.update_op_wfc2 = self.W_fc2.assign(self.update_ph_wfc2)
			self.update_op_bfc1 = self.b_fc1.assign(self.update_ph_bfc1)
			self.update_op_bfc2 = self.b_fc2.assign(self.update_ph_bfc2)
			self.update_op_wol1 = self.W_ol1.assign(self.update_ph_wol1)
			self.update_op_bol1 = self.b_ol1.assign(self.update_ph_bol1)
			
		if NetworkToCopy == None and RestoreFromFile == False:
			self.sess.run(tf.global_variables_initializer()) # Initializes Variables
		
		elif NetworkToCopy != None:
			self.sess.run(tf.global_variables_initializer())			
			self.Copy(NetworkToCopy) # Copies Variables from another Network
			
		elif RestoreFromFile != False:
			self.saver = tf.train.Saver()
			self.saver.restore(self.sess, RestoreFromFile) # RestoreFromFile contains filename

		self.saver = tf.train.Saver()	

				
	def BestAction(self, phy_value):
		deploy_Q = self.sess.run(self.Q_theta, feed_dict={self.phy:phy_value.reshape((-1,self.frame_res*self.nframes))})
		mx = np.max(deploy_Q)
		return np.random.choice(np.where(deploy_Q==mx)[1]) # Returns action with highest Q value
	
	def IterateNetwork(self, phy_value):
		return self.sess.run(self.Q_theta, feed_dict={self.phy:phy_value.reshape((-1,self.frame_res*self.nframes))}) # Returns Q
			
	def TrainNetwork(self, phy_value, action_value, y_value):
		self.sess.run(self.train_step, feed_dict={self.action:action_value, self.y: y_value, self.phy: phy_value})	# Trains Network
		return self.sess.run(self.loss_fct, feed_dict={self.action:action_value, self.y: y_value, self.phy: phy_value})	# Returns the cost function value
	
	def SaveToFile(self, filename):
		self.saver.save(self.sess, filename) # Save all variables to file
	
	def Copy(self, NetworkToCopy):
		self.sess.run(self.update_op_wfc1, {self.update_ph_wfc1: NetworkToCopy.sess.run(NetworkToCopy.W_fc1)})
		self.sess.run(self.update_op_wfc2, {self.update_ph_wfc2: NetworkToCopy.sess.run(NetworkToCopy.W_fc2)})
		self.sess.run(self.update_op_bfc1, {self.update_ph_bfc1: NetworkToCopy.sess.run(NetworkToCopy.b_fc1)})
		self.sess.run(self.update_op_bfc2, {self.update_ph_bfc2: NetworkToCopy.sess.run(NetworkToCopy.b_fc2)})
		self.sess.run(self.update_op_wol1, {self.update_ph_wol1: NetworkToCopy.sess.run(NetworkToCopy.W_ol1)}) 
		self.sess.run(self.update_op_bol1, {self.update_ph_bol1: NetworkToCopy.sess.run(NetworkToCopy.b_ol1)})

