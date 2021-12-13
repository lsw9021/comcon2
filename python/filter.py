import numpy as np

class RunningStat:
	def __init__(self, shape=None):
		self._n = 0
		self._M = np.zeros(shape)
		self._S = np.zeros(shape)

	def copy(self):
		other = RunningStat()
		other._n = self._n
		other._M = np.copy(self._M)
		other._S = np.copy(self._S)
		return other

	def push(self, x):
		x = np.asarray(x)
		# Unvectorized update of the running statistics.
		if x.shape != self._M.shape:
			raise ValueError(
				"Unexpected input shape {}, expected {}, value = {}".format(
					x.shape, self._M.shape, x))
		n1 = self._n
		self._n += 1
		if self._n == 1:
			self._M[...] = x
		else:
			delta = x - self._M
			self._M[...] += delta / self._n
			self._S[...] += delta * delta * n1 / self._n

	def update(self, other):
		n1 = self._n
		n2 = other._n
		n = n1 + n2
		if n == 0:
			# Avoid divide by zero, which creates nans
			return
		delta = self._M - other._M
		delta2 = delta * delta
		M = (n1 * self._M + n2 * other._M) / n
		S = self._S + other._S + delta2 * n1 * n2 / n
		self._n = n
		self._M = M
		self._S = S

	def __repr__(self):
		return "(n={}, mean_mean={}, mean_std={})".format(
			self.n, np.mean(self.mean), np.mean(self.std))

	@property
	def n(self):
		return self._n

	@property
	def mean(self):
		return self._M

	@property
	def var(self):
		return self._S / (self._n - 1) if self._n > 1 else np.square(self._M)

	@property
	def std(self):
		return np.sqrt(self.var)

	@property
	def shape(self):
		return self._M.shape

class MeanStdRuntimeFilter:
	def __init__(self, shape, demean=True, destd=True, clip=10.0):
		self.shape = shape
		self.demean = demean
		self.destd = destd
		self.clip = clip
		self.rs = RunningStat(shape)
		self.std_min = 0.03

	def __call__(self, x, update=True):
		x = np.asarray(x)
		if update:
			if len(x.shape) == len(self.rs.shape) + 1:
				# The vectorized case.
				for i in range(x.shape[0]):
					self.rs.push(x[i])
			else:
				# The unvectorized case.
				self.rs.push(x)
		if self.demean:
			x = x - self.rs.mean
		if self.destd:
			std = np.clip(self.rs.std, self.std_min, 1e8)
			x = x / (std + 1e-8)
		if self.clip:
			x = np.clip(x, -self.clip, self.clip)
		return x

	def unfilter(self, x):
		x = np.asarray(x)
		if self.destd:
			x = x * self.rs.std
		if self.demean:
			x = x + self.rs.mean

		return x
		
	def state_dict(self):
		state = {}
		state['n'] = self.rs._n
		state['M'] = self.rs._M
		state['S'] = self.rs._S

		return state

	def load_state_dict(self, state):
		self.rs._n = state['n']
		self.rs._M = state['M']
		self.rs._S = state['S']
class MeanStdFilter:
	def __init__(self, shape, mean, std):
		self.shape = shape
		self.mean = mean
		self.std = std
		
	def __call__(self, x):
		x = np.asarray(x)
		x = (x - self.mean)/(self.std + 1e-8)
		x = np.clip(x, -10.0, 10.0)
		return x
		
	def state_dict(self):
		state = {}
		state['M'] = self.mean
		state['S'] = self.std
		return state

	def load_state_dict(self, state):
		self.mean = state['M']
		self.std = state['S']
