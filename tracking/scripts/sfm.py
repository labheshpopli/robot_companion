import numpy as np
import sys
import math

class SFMSensor():


## everything can be seen as private
## Externally, one should set robot_position, human_position, robot_velocity
## human_data, and call learn_alphas, and read alphas after that.


	def __init__(self,robot_position,human_position,robot_velocity):

		# Parameters from paper
		self._kappa = 2.3
		self._A = 2.66
		self._B = 0.79
		self._d = 0.4
		self._l = 0.59
                self._alphas = [1.0,1.0,1.0,1.0] ## alpha, beta, gamma, delta
		self.count = 0
		self.SfM = np.array([[0,0]]).T
		self.human_vect = np.array([[0,0]]).T
		self.obs_vect = np.array([[0,0]]).T
		self.target_vect = np.array([[0,0]]).T
		self.robot_vect = np.array([[0,0]]).T
                self.human_robot = np.array([[0,0]]).T
		self._robot_position = robot_position
                self._human_position = human_position
                self._robot_velocity = robot_velocity
                self._human_data = []
                self._old_position = None
		self._lr = 0.5;
		self.updated_position = None;
		self.R = np.array([[1,0.2,0,0],[0.2,1,0,0],
		  					[0,0,1,0.2],[0,0,0.2,1]]);
		self.Q = np.array([[1,0.2],[0.2,1]]);
		self.A = lambda dt: np.array([[1,0,dt,0],[0,1,0,dt],
		  				    [0,0,1,0]      ,[0,0,0,1]]);
		self.B = lambda dt: np.array([[dt **2,0],[0,dt **2],
		  					[dt,0],[0,dt]]);
		self.C = np.array([[1,0,0,0],[0,1,0,0]]);
		self.pred_position = None;
		self.pred_distribution = np.array([[1,0.2,0,0],[0.2,1,0,0],
		  								[0,0,1,0.2],[0,0,0.2,1]]);
		self.updated_distribution = None;


	def predict(self, forces_vect, dt):
		action = np.zeros(forces_vect[0].shape);
		for i in range(len(forces_vect)):
			action += forces_vect[i]*self._alphas[i]
		self.pred_position = np.dot(self.A(dt),self.updated_position) + np.dot(self.B(dt),np.reshape(action,(2,1)));
		self.pred_distribution = np.dot(np.dot(self.A(dt),self.updated_distribution),self.A(dt).T) + self.R;
	
	def update(self):
		if self._robot.stepNum == 1:
			pos = [self._human_position[0],self._human_position[1]];
			pos.append(0)
			pos.append(0)
			self.pred_position = np.reshape(pos,(4,1))
		z = np.reshape(self._human_position,(2,1)) + np.random.rand(2,1)*0.3 - 0.15; ##adding noise
		K = np.dot(np.dot(self.C,self.pred_distribution),self.C.T) + self.Q;
		K = np.linalg.inv(K);
		K = np.dot(np.dot(self.pred_distribution,self.C.T),K);
		self.updated_position = self.pred_position + np.dot(K,z - np.dot(self.C,self.pred_position));
		self.updated_distribution = np.dot(np.identity(4) - np.dot(K,self.C),self.pred_distribution);
	
	def alphas(self,forces):
		diff = self.updated_position - self.pred_position;
		diff = diff[0:2,0];
		diff_norm = math.sqrt(np.inner(diff,diff));
		#diff = diff*5.0/diff_norm if diff_norm > 5.0  else diff
		#print(diff)
		max_f = 0;
		for i,f in enumerate(forces):
			f_norm = np.linalg.norm(f);
			#if i == 0:
			#    print (f)
			if f_norm > max_f:
			  max_f = f_norm;
			f_new = f#f*1.0/f_norm if f_norm > 1.0 else f
			self._alphas[i] = self._alphas[i] + self._lr*np.inner(f_new.flatten(),diff);
		print (self.alpha)#, diff, max_f)
        
        def learn_alphas(self):
                self.calc_sfm()
                self.update()
                forces_vect = [self.target_vect,self.robot_vect,human_vect,obs_vect]
                self.predict(forces_vect)
                self.alpha(forces_vect)
	
        def calc_sfm(self):
		humans = self._create_human_data()
		obstacles = self._create_obstacles_data()
		location = self._human_position
		## we might need to limit te magnitude of this vector
		v_0 = - self._human_position + self._robot_position
		v_0 = v_0*self._robot_velocity/math.sqrt(np.sum(np.power(v_0,2)))
		if self.count == 0:
		        return 0
                else:
			v = location - self._old_position
			theta = math.atan2(v[1],v[0])
			self._old_position = location
		self.force(v_0,v,np.array([location[0],location[1],theta]) ,humans,obstacles)

	def _create_obstacles_data(self):
		## need to read from ray tracing code
		data['distance'] = 0 
	        data['position'] = self._human_position 
		return data

	def _create_human_data(self):
                ## reads human_data which is filled by a method subscribing to the TrackedPersons code
                ## the input from human_data is a list of humans with their positions
                ## even humans that are tracked and not detected are inserted, unless covariance very high
		data = []
		for obj in objects:
			data_object = {}
			data_object['distance'] = np.linalg.norm(obj - self._human_position)
			data_object['position'] = obj 
			data.append(data_object)

		return data

	def forces(self,v_0, v, pose, humans, obs):
		"""
		Parameters:
		v_0 -- a numpy array of the shape (2x1) or (2,)
		v   -- a numpy array of the shape (2x1) or (2,)
		pose -- tuple or list of the shape (x,y,theta) or [x,y,theta]
		humans -- list of objects representing location of other humans
		obstacles -- distance and direction of nearest obstacle
		"""
		# force to target:
		## assuming that target is in direction of motion, with magnitude 0.3 m/s
                force_to_target = self._kappa *v*(0.3-np.linalg.norm(v))
		self.target_vect = np.reshape(force_to_target,(2,1));
		## calculating force to robot 
                force_to_robot = self._kappa *(v_0,v)
		self.robot_vect = np.reshape(force_to_robot,(2,1));
		## initializing human and obstacle vectors
                self.human_vect = np.array([[0,0]], dtype=np.float64).T
		self.obs_vect = np.array([[0,0]], dtype=np.float64).T
		## calculating human forces
                for human in humans:
			pos = human['position']
			dis = human['distance']
			w = self.weight(pose,pos)
			f = self._A * math.exp(self._d - dis)/self._B
			direction = pos - pose[0:2]
			direction = direction /math.sqrt(np.sum(np.power(direction,2)))
			self.human_vect -= np.reshape(f*direction*w,(2,1))
	        ## calculating obstacle force	
                pos = obs['position']
		dis = obs['distance']
		w = self.weight(pose,pos)
		f = self._A * math.exp(self._d - dis)/self._B
		direction = pos - pose[0:2]
		direction = direction /math.sqrt(np.sum(np.power(direction,2)))
		self.obs_vect -= np.reshape(f*direction*w,(2,1))
		
	def weight(self,pose_0, pose_1):
		"""
		returns the weight of the force based on its location
		"""
		phi = abs(math.atan2(pose_1[1] - pose_0[1],pose_1[0]-pose_0[0]) - pose_0[2])
		weight = self._l +(1-self._l)*(1+math.cos(phi))/2
		return weight
