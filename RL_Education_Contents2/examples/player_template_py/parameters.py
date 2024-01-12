class Parameters():
    def __init__(self):

        # simulation parameters
        self.state_type = "state_mix"        # state_absolute/state_relative/state_mix
        self.state_with_friction = False      # True(state + friction)/False(state) 
        self.action_type = "action_position" # action_wheels/action_position
        self.reward_type = "reward_binary"   # reward_continuous/reward_binary/reward_sparse
        self.frame_skip = 10
        self.load = False
        self.play = False

        # agent parameters
        self.num_layers = 1
        self.hidden_dim = 256

        # ddpg parameters
        self.ddpg_gamma = 0.99
        self.ddpg_batch_size = 256
        self.ddpg_buffer_size = 20000
        self.ddpg_actor_lr = 0.0001
        self.ddpg_critic_lr = 0.001
        self.ddpg_grad_norm_clip = 10
        self.ddpg_dec_exploration = 10000
