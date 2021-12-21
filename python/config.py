config = {
	'save_path' : '/home/seunghwan/Documents/comcon2/data/learning/',
	'sample_size' : 2048,
	'num_sgd_iter' : 5,
	'sgd_minibatch_size' : 128,
	
	'num_disc_sgd_iter' : 1,
	'disc_sgd_minibatch_size' : 16,
	'save_frequency' : [10, 200],
	'force_function_update_frequency' : 10,
	'policy' : {
		'hidden':[256,256],
		'std' : 0.1,
		'gamma' : 0.95,
		'lb' : 0.95,
		'policy_lr' : 1e-5,
		'value_lr' : 1e-4,
		'policy_clip' : 0.2,
		'value_clip' : 1.0,
		'grad_clip' : 0.5,
		'kl' : 0.01,
		'entropy' : 0.0
	},
	'disc' :{
		'hidden':[256,256],
		'w_grad':10.0,
		'w_reg':0.05,
		'w_decay':0.0005,
		'r_scale':2.0,
		'grad_clip':0.5,
		'lr':0.00001
	}
}