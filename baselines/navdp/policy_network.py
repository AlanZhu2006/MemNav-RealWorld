import torch
import torch.nn as nn
import math
import numpy as np
import torch.nn.functional as F
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from policy_backbone import *

class NavDP_Policy(nn.Module):
    def __init__(self,
                 image_size=224,
                 memory_size=8,
                 predict_size=24,
                 temporal_depth=8,
                 heads=8,
                 token_dim=384,
                 channels=3,
                 device='cuda:0'):
        super().__init__()
        self.device = device
        self.image_size = image_size
        self.memory_size = memory_size
        self.predict_size = predict_size
        self.temporal_depth = temporal_depth
        self.attention_heads = heads
        self.input_channels = channels
        self.token_dim = token_dim

        # input encoders
        self.rgbd_encoder = NavDP_RGBD_Backbone(image_size,token_dim,memory_size=memory_size,device=device)
        self.point_encoder = nn.Linear(3,self.token_dim)
        self.pixel_encoder = NavDP_PixelGoal_Backbone(image_size,token_dim,device=device)
        self.image_encoder = NavDP_ImageGoal_Backbone(image_size,token_dim,device=device)

        # fusion layers
        self.decoder_layer = nn.TransformerDecoderLayer(d_model = token_dim,
                                                        nhead = heads,
                                                        dim_feedforward = 4 * token_dim,
                                                        activation = 'gelu',
                                                        batch_first = True,
                                                        norm_first = True)
        self.decoder = nn.TransformerDecoder(decoder_layer = self.decoder_layer,
                                             num_layers = self.temporal_depth)

        self.input_embed = nn.Linear(3,token_dim) # encode the actions for denoise/critic
        self.cond_pos_embed = LearnablePositionalEncoding(token_dim, memory_size * 16 + 4) # time,point,image,pixel,input
        self.out_pos_embed = LearnablePositionalEncoding(token_dim, predict_size)
        self.time_emb = SinusoidalPosEmb(token_dim)
        self.layernorm = nn.LayerNorm(token_dim)

        self.action_head = nn.Linear(token_dim, 3)
        self.critic_head = nn.Linear(token_dim, 1)
        self.noise_scheduler = DDPMScheduler(num_train_timesteps=10,
                                       beta_schedule='squaredcos_cap_v2',
                                       clip_sample=True,
                                       prediction_type='epsilon')

        self.tgt_mask = (torch.triu(torch.ones(predict_size, predict_size)) == 1).transpose(0, 1)
        self.tgt_mask = self.tgt_mask.float().masked_fill(self.tgt_mask == 0, float('-inf')).masked_fill(self.tgt_mask == 1, float(0.0))
        self.cond_critic_mask = torch.zeros((predict_size,4 + memory_size * 16))
        self.cond_critic_mask[:,0:4] = float('-inf')

        # Training-only heads are unused by inference, but retaining them makes
        # the frozen checkpoint architecture exact instead of silently dropping
        # four tensors under ``strict=False``.
        self.pixel_aux_head = nn.Linear(self.token_dim, 3)
        self.image_aux_head = nn.Linear(self.token_dim, 3)

    def predict_noise(self,last_actions,timestep,goal_embed,rgbd_embed):
        action_embeds = self.input_embed(last_actions)
        time_embeds = self.time_emb(timestep.to(self.device)).unsqueeze(1).tile((last_actions.shape[0],1,1))
        cond_embedding = torch.cat([time_embeds,goal_embed,goal_embed,goal_embed,rgbd_embed],dim=1) + self.cond_pos_embed(torch.cat([time_embeds,goal_embed,goal_embed,goal_embed,rgbd_embed],dim=1))
        input_embedding = action_embeds + self.out_pos_embed(action_embeds)
        output = self.decoder(tgt = input_embedding,memory = cond_embedding, tgt_mask = self.tgt_mask.to(self.device))
        output = self.layernorm(output)
        output = self.action_head(output)
        return output

    def predict_mix_noise(self,last_actions,timestep,goal_embeds,rgbd_embed):
        action_embeds = self.input_embed(last_actions)
        time_embeds = self.time_emb(timestep.to(self.device)).unsqueeze(1).tile((last_actions.shape[0],1,1))
        cond_embedding = torch.cat([time_embeds,goal_embeds[0],goal_embeds[1],goal_embeds[2],rgbd_embed],dim=1) + self.cond_pos_embed(torch.cat([time_embeds,goal_embeds[0],goal_embeds[1],goal_embeds[2],rgbd_embed],dim=1))
        input_embedding = action_embeds + self.out_pos_embed(action_embeds)
        output = self.decoder(tgt = input_embedding,memory = cond_embedding, tgt_mask = self.tgt_mask.to(self.device))
        output = self.layernorm(output)
        output = self.action_head(output)
        return output

    def score_imagegoal_trajectories(self, goal_image, input_images,
                                     input_depths, trajectories, *,
                                     control_goal_image=None,
                                     timesteps=None, noise_samples=1,
                                     seed=0):
        """Score fixed trajectories with paired ImageGoal denoising errors.

        This is a read-only diagnostic.  It does not sample an action and does
        not modify the diffusion scheduler or the agent observation FIFO.
        For every candidate, timestep, and Monte-Carlo repeat, the conditioned
        and zero-goal denoisers see exactly the same noised trajectory.  The
        returned ``goal_advantage`` is ``MSE(null) - MSE(goal)``; it is a
        denoising-error contrast, not a calibrated trajectory likelihood.

        ``trajectories`` are NavDP's post-processed cumulative waypoints.  They
        are converted back to the action increments used to train the denoiser
        before scoring.
        """
        with torch.no_grad():
            trajectory = torch.as_tensor(
                trajectories, dtype=torch.float32, device=self.device)
            if trajectory.ndim != 4 or trajectory.shape[-1] != 3:
                raise ValueError(
                    "trajectories must have shape [batch, candidates, time, 3]")
            batch_size, candidate_count, horizon, _ = trajectory.shape
            if horizon != self.predict_size:
                raise ValueError(
                    f"trajectory horizon {horizon} != predict_size "
                    f"{self.predict_size}")
            if int(goal_image.shape[0]) != batch_size:
                raise ValueError("goal/trajectory batch sizes differ")
            if int(input_images.shape[0]) != batch_size:
                raise ValueError("observation/trajectory batch sizes differ")
            if control_goal_image is not None:
                if tuple(control_goal_image.shape) != tuple(goal_image.shape):
                    raise ValueError(
                        "control goal must have the same shape as the goal")

            if timesteps is None:
                timesteps = tuple(range(
                    int(self.noise_scheduler.config.num_train_timesteps)))
            else:
                timesteps = tuple(int(value) for value in timesteps)
            if not timesteps or len(set(timesteps)) != len(timesteps):
                raise ValueError("timesteps must be a non-empty unique sequence")
            max_timestep = int(
                self.noise_scheduler.config.num_train_timesteps) - 1
            if any(value < 0 or value > max_timestep for value in timesteps):
                raise ValueError(
                    f"timesteps must lie in [0, {max_timestep}]")
            noise_samples = int(noise_samples)
            if noise_samples < 1:
                raise ValueError("noise_samples must be positive")

            # NavDP exposes cumulative waypoints but trains its diffusion head
            # on per-step actions scaled by four.
            origin = torch.zeros_like(trajectory[:, :, :1])
            previous = torch.cat([origin, trajectory[:, :, :-1]], dim=2)
            clean_actions = ((trajectory - previous) * 4.0).reshape(
                batch_size * candidate_count, horizon, 3)

            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            current_image = input_images[:, -1]
            imagegoal_embed = self.image_encoder(
                np.concatenate((goal_image, current_image), axis=-1)
            ).unsqueeze(1)
            control_embed = None
            if control_goal_image is not None:
                control_embed = self.image_encoder(
                    np.concatenate(
                        (control_goal_image, current_image), axis=-1)
                ).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(
                rgbd_embed, candidate_count, dim=0)
            imagegoal_embed = torch.repeat_interleave(
                imagegoal_embed, candidate_count, dim=0)
            nogoal_embed = torch.zeros_like(imagegoal_embed)
            if control_embed is not None:
                control_embed = torch.repeat_interleave(
                    control_embed, candidate_count, dim=0)

            score_shape = (batch_size, candidate_count)
            goal_mse = torch.zeros(score_shape, device=self.device)
            nogoal_mse = torch.zeros(score_shape, device=self.device)
            control_mse = (
                torch.zeros(score_shape, device=self.device)
                if control_embed is not None else None)
            per_timestep_goal = []
            per_timestep_control = []

            generator = torch.Generator(device=torch.device(self.device))
            generator.manual_seed(int(seed))
            for timestep_value in timesteps:
                timestep_goal = torch.zeros(score_shape, device=self.device)
                timestep_nogoal = torch.zeros(score_shape, device=self.device)
                timestep_control = (
                    torch.zeros(score_shape, device=self.device)
                    if control_embed is not None else None)
                timestep_batch = torch.full(
                    (batch_size * candidate_count,), timestep_value,
                    dtype=torch.long, device=self.device)
                model_timestep = torch.as_tensor(
                    [timestep_value], dtype=torch.long, device=self.device)
                for _ in range(noise_samples):
                    # A single noise draw per environment is shared by every
                    # candidate and every conditioning arm.
                    base_noise = torch.randn(
                        (batch_size, 1, horizon, 3), generator=generator,
                        device=self.device)
                    noise = base_noise.expand(
                        -1, candidate_count, -1, -1).reshape(
                            batch_size * candidate_count, horizon, 3)
                    noisy_actions = self.noise_scheduler.add_noise(
                        clean_actions, noise, timestep_batch)

                    goal_prediction = self.predict_noise(
                        noisy_actions, model_timestep, imagegoal_embed,
                        rgbd_embed)
                    nogoal_prediction = self.predict_noise(
                        noisy_actions, model_timestep, nogoal_embed,
                        rgbd_embed)
                    timestep_goal += F.mse_loss(
                        goal_prediction, noise, reduction='none'
                    ).mean(dim=(1, 2)).reshape(score_shape)
                    timestep_nogoal += F.mse_loss(
                        nogoal_prediction, noise, reduction='none'
                    ).mean(dim=(1, 2)).reshape(score_shape)
                    if control_embed is not None:
                        control_prediction = self.predict_noise(
                            noisy_actions, model_timestep, control_embed,
                            rgbd_embed)
                        timestep_control += F.mse_loss(
                            control_prediction, noise, reduction='none'
                        ).mean(dim=(1, 2)).reshape(score_shape)

                timestep_goal /= noise_samples
                timestep_nogoal /= noise_samples
                goal_mse += timestep_goal
                nogoal_mse += timestep_nogoal
                per_timestep_goal.append(
                    (timestep_nogoal - timestep_goal).cpu().tolist())
                if control_mse is not None:
                    timestep_control /= noise_samples
                    control_mse += timestep_control
                    per_timestep_control.append(
                        (timestep_nogoal - timestep_control).cpu().tolist())

            denominator = float(len(timesteps))
            goal_mse /= denominator
            nogoal_mse /= denominator
            goal_advantage = nogoal_mse - goal_mse
            normalized_advantage = goal_advantage / nogoal_mse.clamp_min(1e-8)
            result = {
                "goal_mse": goal_mse.cpu().tolist(),
                "nogoal_mse": nogoal_mse.cpu().tolist(),
                "goal_advantage": goal_advantage.cpu().tolist(),
                "normalized_goal_advantage": (
                    normalized_advantage.cpu().tolist()),
                "per_timestep_goal_advantage": per_timestep_goal,
                "timesteps": list(timesteps),
                "noise_samples": noise_samples,
                "score_seed": int(seed),
                "shared_noise_across_candidates": True,
                "score_semantics": "nogoal_mse_minus_goal_mse",
                "is_calibrated_likelihood": False,
            }
            if control_mse is not None:
                control_mse /= denominator
                control_advantage = nogoal_mse - control_mse
                result.update({
                    "control_goal_mse": control_mse.cpu().tolist(),
                    "control_goal_advantage": (
                        control_advantage.cpu().tolist()),
                    "goal_vs_control_advantage": (
                        (goal_advantage - control_advantage).cpu().tolist()),
                    "per_timestep_control_goal_advantage": (
                        per_timestep_control),
                })
            return result

    def predict_critic(self,predict_trajectory,rgbd_embed):
        nogoal_embed = torch.zeros_like(rgbd_embed[:,0:1])
        action_embeddings = self.input_embed(predict_trajectory)
        action_embeddings = action_embeddings + self.out_pos_embed(action_embeddings)
        cond_embeddings = torch.cat([nogoal_embed,nogoal_embed,nogoal_embed,nogoal_embed,rgbd_embed],dim=1) +  self.cond_pos_embed(torch.cat([nogoal_embed,nogoal_embed,nogoal_embed,nogoal_embed,rgbd_embed],dim=1))
        critic_output = self.decoder(tgt = action_embeddings, memory = cond_embeddings, memory_mask = self.cond_critic_mask.to(self.device))
        critic_output = self.layernorm(critic_output)
        critic_output = self.critic_head(critic_output.mean(dim=1))[:,0]
        return critic_output

    def predict_pointgoal_action(self,goal_point,input_images,input_depths,sample_num=16):
        with torch.no_grad():
            tensor_point_goal = torch.as_tensor(goal_point,dtype=torch.float32,device=self.device)
            rgbd_embed = self.rgbd_encoder(input_images,input_depths)
            pointgoal_embed = self.point_encoder(tensor_point_goal).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(rgbd_embed,sample_num,dim=0)
            pointgoal_embed = torch.repeat_interleave(pointgoal_embed,sample_num,dim=0)

            noisy_action = torch.randn((sample_num * goal_point.shape[0], self.predict_size, 3), device=self.device)
            naction = noisy_action
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for k in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_noise(naction,k.unsqueeze(0),pointgoal_embed,rgbd_embed)
                naction = self.noise_scheduler.step(model_output=noise_pred,timestep=k,sample=naction).prev_sample

            critic_values = self.predict_critic(naction,rgbd_embed)
            critic_values = critic_values.reshape(goal_point.shape[0],sample_num)

            all_trajectory = torch.cumsum(naction / 4.0, dim=1)
            all_trajectory = all_trajectory.reshape(goal_point.shape[0],sample_num,self.predict_size,3)
            trajectory_length = all_trajectory[:,:,-1,0:2].norm(dim=-1)
            all_trajectory[trajectory_length < 0.5] = all_trajectory[trajectory_length < 0.5] * torch.tensor([[[0,0,1.0]]],device=all_trajectory.device)

            sorted_indices = (-critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_point.shape[0]).unsqueeze(1).expand(-1, 2)
            positive_trajectory = all_trajectory[batch_indices, topk_indices]

            sorted_indices = (critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_point.shape[0]).unsqueeze(1).expand(-1, 2)
            negative_trajectory = all_trajectory[batch_indices, topk_indices]

            return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), positive_trajectory.cpu().numpy(), negative_trajectory.cpu().numpy()

    def predict_imagegoal_action(self,goal_image,input_images,input_depths,sample_num=16):
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images,input_depths)
            imagegoal_embed = self.image_encoder(np.concatenate((goal_image,input_images[:,-1]),axis=-1)).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(rgbd_embed,sample_num,dim=0)
            imagegoal_embed = torch.repeat_interleave(imagegoal_embed,sample_num,dim=0)

            noisy_action = torch.randn((sample_num * goal_image.shape[0], self.predict_size, 3), device=self.device)
            naction = noisy_action
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for k in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_noise(naction,k.unsqueeze(0),imagegoal_embed,rgbd_embed)
                naction = self.noise_scheduler.step(model_output=noise_pred,timestep=k,sample=naction).prev_sample

            critic_values = self.predict_critic(naction,rgbd_embed)
            critic_values = critic_values.reshape(goal_image.shape[0],sample_num)

            all_trajectory = torch.cumsum(naction / 4.0, dim=1)
            all_trajectory = all_trajectory.reshape(goal_image.shape[0],sample_num,self.predict_size,3)
            trajectory_length = all_trajectory[:,:,-1,0:2].norm(dim=-1)
            all_trajectory[trajectory_length < 0.5] = all_trajectory[trajectory_length < 0.5] * torch.tensor([[[0,0,1.0]]],device=all_trajectory.device)

            sorted_indices = (-critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            positive_trajectory = all_trajectory[batch_indices, topk_indices]

            sorted_indices = (critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            negative_trajectory = all_trajectory[batch_indices, topk_indices]

            return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), positive_trajectory.cpu().numpy(), negative_trajectory.cpu().numpy()

    def predict_pixelgoal_action(self,goal_image,input_images,input_depths,sample_num=16):
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images,input_depths)
            pixelgoal_embed = self.pixel_encoder(np.concatenate((goal_image[:,:,:,None],input_images[:,-1]),axis=-1)).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(rgbd_embed,sample_num,dim=0)
            pixelgoal_embed = torch.repeat_interleave(pixelgoal_embed,sample_num,dim=0)

            noisy_action = torch.randn((sample_num * goal_image.shape[0], self.predict_size, 3), device=self.device)
            naction = noisy_action
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for k in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_noise(naction,k.unsqueeze(0),pixelgoal_embed,rgbd_embed)
                naction = self.noise_scheduler.step(model_output=noise_pred,timestep=k,sample=naction).prev_sample

            critic_values = self.predict_critic(naction,rgbd_embed)
            critic_values = critic_values.reshape(goal_image.shape[0],sample_num)

            all_trajectory = torch.cumsum(naction / 4.0, dim=1)
            all_trajectory = all_trajectory.reshape(goal_image.shape[0],sample_num,self.predict_size,3)
            trajectory_length = all_trajectory[:,:,-1,0:2].norm(dim=-1)
            all_trajectory[trajectory_length < 0.5] = all_trajectory[trajectory_length < 0.5] * torch.tensor([[[0,0,1.0]]],device=all_trajectory.device)

            sorted_indices = (-critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            positive_trajectory = all_trajectory[batch_indices, topk_indices]

            sorted_indices = (critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            negative_trajectory = all_trajectory[batch_indices, topk_indices]

            return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), positive_trajectory.cpu().numpy(), negative_trajectory.cpu().numpy()

    def predict_nogoal_action(self,input_images,input_depths,sample_num=16):
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images,input_depths)
            nogoal_embed = torch.zeros_like(rgbd_embed[:,0:1])
            rgbd_embed = torch.repeat_interleave(rgbd_embed,sample_num,dim=0)
            nogoal_embed = torch.repeat_interleave(nogoal_embed,sample_num,dim=0)

            noisy_action = torch.randn((sample_num * input_images.shape[0], self.predict_size, 3), device=self.device)
            naction = noisy_action
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for k in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_noise(naction,k.unsqueeze(0),nogoal_embed,rgbd_embed)
                naction = self.noise_scheduler.step(model_output=noise_pred,timestep=k,sample=naction).prev_sample

            critic_values = self.predict_critic(naction,rgbd_embed)
            critic_values = critic_values.reshape(input_images.shape[0],sample_num)

            all_trajectory = torch.cumsum(naction / 4.0, dim=1)
            all_trajectory = all_trajectory.reshape(input_images.shape[0],sample_num,self.predict_size,3)

            #distance = all_trajectory[:,-1,0:2].square().sum(dim=-1).sqrt()
            #critic_values[torch.where(distance<0.5)[0]] = -10.0
            #all_trajectory[trajectory_length < 0.5] = all_trajectory[trajectory_length < 0.5] * torch.tensor([[[0,0,1.0]]],device=all_trajectory.device)
            trajectory_length = all_trajectory[:,:,-1,0:2].norm(dim=-1)
            print(trajectory_length.shape,trajectory_length.max(),trajectory_length.min())
            critic_values[torch.where(trajectory_length<1.0)] -= 10.0

            sorted_indices = (-critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(input_images.shape[0]).unsqueeze(1).expand(-1, 2)
            positive_trajectory = all_trajectory[batch_indices, topk_indices]

            sorted_indices = (critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(input_images.shape[0]).unsqueeze(1).expand(-1, 2)
            negative_trajectory = all_trajectory[batch_indices, topk_indices]

            #import pdb
            #pdb.set_trace()

            return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), positive_trajectory.cpu().numpy(), negative_trajectory.cpu().numpy()

    def predict_ip_action(self,goal_point,goal_image,input_images,input_depths,sample_num=16):
        with torch.no_grad():
            tensor_point_goal = torch.as_tensor(goal_point,dtype=torch.float32,device=self.device)
            rgbd_embed = self.rgbd_encoder(input_images,input_depths)
            imagegoal_embed = self.image_encoder(np.concatenate((goal_image,input_images[:,-1]),axis=-1)).unsqueeze(1)
            pointgoal_embed = self.point_encoder(tensor_point_goal).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(rgbd_embed,sample_num,dim=0)
            pointgoal_embed = torch.repeat_interleave(pointgoal_embed,sample_num,dim=0)
            imagegoal_embed = torch.repeat_interleave(imagegoal_embed,sample_num,dim=0)

            noisy_action = torch.randn((sample_num * goal_image.shape[0], self.predict_size, 3), device=self.device)
            naction = noisy_action
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for k in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_mix_noise(naction,k.unsqueeze(0),[imagegoal_embed,pointgoal_embed,imagegoal_embed],rgbd_embed)
                naction = self.noise_scheduler.step(model_output=noise_pred,timestep=k,sample=naction).prev_sample

            critic_values = self.predict_critic(naction,rgbd_embed)
            critic_values = critic_values.reshape(goal_image.shape[0],sample_num)

            all_trajectory = torch.cumsum(naction / 4.0, dim=1)
            all_trajectory = all_trajectory.reshape(goal_image.shape[0],sample_num,self.predict_size,3)
            trajectory_length = all_trajectory[:,:,-1,0:2].norm(dim=-1)
            all_trajectory[trajectory_length < 0.5] = all_trajectory[trajectory_length < 0.5] * torch.tensor([[[0,0,1.0]]],device=all_trajectory.device)

            sorted_indices = (-critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            positive_trajectory = all_trajectory[batch_indices, topk_indices]

            sorted_indices = (critic_values).argsort(dim=1)
            topk_indices = sorted_indices[:,0:2]
            batch_indices = torch.arange(goal_image.shape[0]).unsqueeze(1).expand(-1, 2)
            negative_trajectory = all_trajectory[batch_indices, topk_indices]

            return all_trajectory.cpu().numpy(), critic_values.cpu().numpy(), positive_trajectory.cpu().numpy(), negative_trajectory.cpu().numpy()
