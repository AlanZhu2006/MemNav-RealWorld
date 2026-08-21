import torch
import numpy as np
import cv2
from PIL import Image
from matplotlib import colormaps as cm
from policy_network import NavDP_Policy

class NavDP_Agent:
    def __init__(self,
                 image_intrinsic,
                 image_size=224,
                 memory_size=8,
                 predict_size=24,
                 temporal_depth=16,
                 heads=8,
                 token_dim=384,
                 navi_model = "./100.ckpt",
                 device='cuda:0'):
        self.image_intrinsic = image_intrinsic
        self.device = device
        self.predict_size = predict_size
        self.image_size = image_size
        self.memory_size = memory_size
        self.navi_former = NavDP_Policy(image_size,memory_size,predict_size,temporal_depth,heads,token_dim,device)
        checkpoint = torch.load(navi_model, map_location=self.device)
        incompatible = self.navi_former.load_state_dict(
            checkpoint, strict=False)
        if incompatible.missing_keys or incompatible.unexpected_keys:
            raise RuntimeError(
                "NavDP checkpoint/model contract mismatch: missing={} "
                "unexpected={}".format(
                    incompatible.missing_keys, incompatible.unexpected_keys))
        self.checkpoint_contract = {
            "exact_state_dict": True,
            "parameter_tensor_count": int(len(checkpoint)),
            "temporal_depth": int(temporal_depth),
        }
        self.navi_former.to(self.device)
        self.navi_former.eval()

    def reset(self,batch_size,threshold):
        self.batch_size = batch_size
        self.stop_threshold = threshold
        self.memory_queue = [[] for i in range(batch_size)]
    def reset_env(self,i):
        self.memory_queue[i] = []

    def append_observation(self, images):
        """Restore a decision observation without sampling an action.

        This is used only by the paired evaluator when replaying a previously
        frozen Novel rollout.  It follows the exact RGB queue update used by
        every normal NavDP step and deliberately does not consume DDPM RNG.
        """
        process_images = self.process_image(images)
        if len(process_images) != len(self.memory_queue):
            raise ValueError("observation batch differs from NavDP batch size")
        for index, processed in enumerate(process_images):
            queue = self.memory_queue[index]
            if len(queue) >= self.memory_size:
                del queue[0]
            queue.append(processed)
        return [len(queue) for queue in self.memory_queue]

    def project_trajectory(self,images,n_trajectories,n_values):
        trajectory_masks = []
        for i in range(images.shape[0]):
            trajectory_mask = np.array(images[i])
            n_trajectory = n_trajectories[i,:,:,0:2]
            n_value = n_values[i]
            for waypoints,value in zip(n_trajectory,n_value):
                norm_value = np.clip(-value*0.1,0,1)
                colormap = cm.get('jet')
                color = np.array(colormap(norm_value)[0:3]) * 255.0
                input_points = np.zeros((waypoints.shape[0],3)) - 0.2
                input_points[:,0:2] = waypoints
                input_points[:,1] = -input_points[:,1]
                camera_z = images[0].shape[0] - 1 - self.image_intrinsic[1][1] * input_points[:,2] / (input_points[:,0] + 1e-8) - self.image_intrinsic[1][2]
                camera_x = self.image_intrinsic[0][0] * input_points[:,1] / (input_points[:,0] + 1e-8) + self.image_intrinsic[0][2]
                for i in range(camera_x.shape[0]-1):
                    try:
                        if camera_x[i] > 0 and camera_z[i] > 0 and camera_x[i+1] > 0 and camera_z[i+1] > 0:
                            trajectory_mask = cv2.line(trajectory_mask,(int(camera_x[i]),int(camera_z[i])),(int(camera_x[i+1]),int(camera_z[i+1])),color.astype(np.uint8).tolist(),5)
                    except:
                        pass
            trajectory_masks.append(trajectory_mask)
        return np.concatenate(trajectory_masks,axis=1)

    def process_image(self,images):
        assert len(images.shape) == 4
        H,W,C = images.shape[1],images.shape[2],images.shape[3]
        prop = self.image_size/max(H,W)
        return_images = []
        for img in images:
            resize_image = cv2.resize(img,(-1,-1),fx=prop,fy=prop)
            pad_width = max((self.image_size - resize_image.shape[1])//2,0)
            pad_height = max((self.image_size - resize_image.shape[0])//2,0)
            pad_image = np.pad(resize_image,((pad_height,pad_height),(pad_width,pad_width),(0,0)),mode='constant',constant_values=0)
            resize_image = cv2.resize(pad_image,(self.image_size,self.image_size))
            resize_image = np.array(resize_image)
            resize_image = resize_image.astype(np.float32) / 255.0
            return_images.append(resize_image)
        return np.array(return_images)

    def process_depth(self,depths):
        assert len(depths.shape) == 4
        depths[depths==np.inf] = 0
        H,W,C = depths.shape[1],depths.shape[2],depths.shape[3]
        prop = self.image_size/max(H,W)
        return_depths = []
        for depth in depths:
            resize_depth = cv2.resize(depth,(-1,-1),fx=prop,fy=prop)
            pad_width = max((self.image_size - resize_depth.shape[1])//2,0)
            pad_height = max((self.image_size - resize_depth.shape[0])//2,0)
            pad_depth = np.pad(resize_depth,((pad_height,pad_height),(pad_width,pad_width)),mode='constant',constant_values=0)
            resize_depth = cv2.resize(pad_depth,(self.image_size,self.image_size))
            resize_depth[resize_depth>5.0] = 0
            resize_depth[resize_depth<0.1] = 0
            return_depths.append(resize_depth[:,:,np.newaxis])
        return np.array(return_depths)

    def process_pixel(self,pixel_coords,input_images):
        return_pixels = []
        H,W,C = input_images.shape[1],input_images.shape[2],input_images.shape[3]
        prop = self.image_size/max(H,W)
        for pixel_coord,input_image in zip(pixel_coords,input_images):
            panel_image = np.zeros_like(input_image,dtype=np.uint8)
            min_x = pixel_coord[0] - 10
            min_y = pixel_coord[1] - 10
            max_x = pixel_coord[0] + 10
            max_y = pixel_coord[1] + 10

            if min_x <= 0:
                panel_image[:,0:10] = 255
            elif min_y <= 0:
                panel_image[0:10,:] = 255
            elif max_x >= panel_image.shape[1]:
                panel_image[:,panel_image.shape[1]-10:] = 255
            elif max_y >= panel_image.shape[0]:
                panel_image[panel_image.shape[0]-10:,:] = 255
            elif min_x > 0 and min_y > 0 and max_x < panel_image.shape[1] and max_y < panel_image.shape[0]:
                panel_image[min_y:max_y,min_x:max_x] = 255

            resize_image = cv2.resize(panel_image,(-1,-1),fx=prop,fy=prop, interpolation=cv2.INTER_NEAREST)
            pad_width = max((self.image_size - resize_image.shape[1])//2,0)
            pad_height = max((self.image_size - resize_image.shape[0])//2,0)
            pad_image = np.pad(resize_image,((pad_height,pad_height),(pad_width,pad_width),(0,0)),mode='constant',constant_values=0)
            resize_image = cv2.resize(pad_image,(self.image_size,self.image_size))
            resize_image = np.array(resize_image)
            resize_image = resize_image.astype(np.float32) / 255.0
            return_pixels.append(resize_image)
        return np.array(return_pixels).mean(axis=-1)

    def process_pointgoal(self,goals):
        clip_goals = goals.clip(-10,10)
        clip_goals[:,0] = np.clip(clip_goals[:,0],0,10)
        return clip_goals

    def step_nogoal(self,images,depths):
        process_images = self.process_image(images)
        process_depths = self.process_depth(depths)
        input_images = []
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size:
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
                input_image = np.pad(input_image,((self.memory_size - input_image.shape[0],0),(0,0),(0,0),(0,0)))
            else:
                del self.memory_queue[i][0]
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])

            input_images.append(input_image)
        input_image = np.array(input_images)
        input_depth = process_depths
        # cv2.imwrite("input_image.jpg",np.concatenate(self.memory_queue[0],axis=0)*255)
        all_trajectory, all_values, good_trajectory, bad_trajectory = self.navi_former.predict_nogoal_action(input_image,input_depth)
        if all_values.max() < self.stop_threshold:
            good_trajectory[:,:,:,0] = good_trajectory[:,:,:,0] * 0.0
            good_trajectory[:,:,:,1] = np.sign(good_trajectory[:,:,:,1].mean())
        trajectory_mask = self.project_trajectory(images,all_trajectory,all_values)
        return good_trajectory[:,0], all_trajectory, all_values, trajectory_mask

    def step_pointgoal(self,goals,images,depths):
        process_images = self.process_image(images)
        process_depths = self.process_depth(depths)
        input_images = []
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size:
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
                input_image = np.pad(input_image,((self.memory_size - input_image.shape[0],0),(0,0),(0,0),(0,0)))
            else:
                del self.memory_queue[i][0]
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])

            input_images.append(input_image)
        input_image = np.array(input_images)
        input_depth = process_depths
        input_goals = self.process_pointgoal(goals)
        # cv2.imwrite("input_image.jpg",np.concatenate(self.memory_queue[0],axis=0)*255)
        all_trajectory, all_values, good_trajectory, bad_trajectory = self.navi_former.predict_pointgoal_action(input_goals,input_image,input_depth)
        if all_values.max() < self.stop_threshold:
            good_trajectory[:,:,:,0] = good_trajectory[:,:,:,0] * 0.0
            good_trajectory[:,:,:,1] = np.sign(good_trajectory[:,:,:,1].mean())

        print(all_values.max(),all_values.min())

        trajectory_mask = self.project_trajectory(images,all_trajectory,all_values)
        return good_trajectory[:,0], all_trajectory, all_values, trajectory_mask

    def step_imagegoal(self,goals,images,depths):
        process_images = self.process_image(images)
        process_depths = self.process_depth(depths)
        input_images = []
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size:
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
                input_image = np.pad(input_image,((self.memory_size - input_image.shape[0],0),(0,0),(0,0),(0,0)))
            else:
                del self.memory_queue[i][0]
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
            input_images.append(input_image)
        input_image = np.array(input_images)
        input_depth = process_depths
        input_goals = self.process_image(goals)
        # cv2.imwrite("input_image.jpg",np.concatenate(self.memory_queue[0],axis=0)*255)
        all_trajectory, all_values, good_trajectory, bad_trajectory = self.navi_former.predict_imagegoal_action(input_goals,input_image,input_depth)
        if all_values.max() < self.stop_threshold:
            good_trajectory[:,:,:,0] = good_trajectory[:,:,:,0] * 0.0
            good_trajectory[:,:,:,1] = np.sign(good_trajectory[:,:,:,1].mean())

        print(all_values.max(),all_values.min())
        trajectory_mask = self.project_trajectory(images,all_trajectory,all_values)
        return good_trajectory[:,0], all_trajectory, all_values, trajectory_mask

    def resample_imagegoal(self, goals, images, depths):
        """Sample new ImageGoal candidates without advancing observation state.

        A normal ``step_imagegoal`` must run first for the current decision
        frame.  This diagnostic path then reuses that exact FIFO so additional
        diffusion seeds cannot duplicate the current image or change future
        policy inputs.
        """
        if len(images) != len(self.memory_queue):
            raise ValueError("observation batch differs from NavDP batch size")
        input_images = []
        for queue in self.memory_queue:
            if not queue:
                raise RuntimeError(
                    "ImageGoal resampling requires a prior policy step")
            input_image = np.array(queue)
            if input_image.shape[0] < self.memory_size:
                input_image = np.pad(
                    input_image,
                    ((self.memory_size - input_image.shape[0], 0),
                     (0, 0), (0, 0), (0, 0)),
                )
            input_images.append(input_image)
        input_image = np.array(input_images)
        input_depth = self.process_depth(depths)
        input_goals = self.process_image(goals)
        all_trajectory, all_values, good_trajectory, bad_trajectory = (
            self.navi_former.predict_imagegoal_action(
                input_goals, input_image, input_depth))
        if all_values.max() < self.stop_threshold:
            good_trajectory[:, :, :, 0] = 0.0
            good_trajectory[:, :, :, 1] = np.sign(
                good_trajectory[:, :, :, 1].mean())
        trajectory_mask = self.project_trajectory(
            images, all_trajectory, all_values)
        return good_trajectory[:, 0], all_trajectory, all_values, trajectory_mask

    def step_pixelgoal(self,goals,images,depths):
        process_images = self.process_image(images)
        process_depths = self.process_depth(depths)
        input_images = []
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size:
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
                input_image = np.pad(input_image,((self.memory_size - input_image.shape[0],0),(0,0),(0,0),(0,0)))
            else:
                del self.memory_queue[i][0]
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
            input_images.append(input_image)

        input_image = np.array(input_images)
        input_depth = process_depths
        input_goals = self.process_pixel(goals,images)

        # cv2.imwrite("input_image.jpg",np.concatenate(self.memory_queue[0],axis=0)*255)
        pixel_vis_image = input_image[0,-1].copy() * 255
        pixel_vis_image[np.where(input_goals[0]==1)] = np.array([255,0,0])
        # cv2.imwrite("pixel_goal.jpg",pixel_vis_image)
        all_trajectory, all_values, good_trajectory, bad_trajectory = self.navi_former.predict_pixelgoal_action(input_goals,input_image,input_depth)

        if all_values.max() < self.stop_threshold:
            good_trajectory[:,:,:,0] = good_trajectory[:,:,:,0] * 0.0
            good_trajectory[:,:,:,1] = np.sign(good_trajectory[:,:,:,1].mean())

        trajectory_mask = self.project_trajectory(images,all_trajectory,all_values)
        return good_trajectory[:,0], all_trajectory, all_values, trajectory_mask

    def step_point_image_goal(self,pointgoal,imagegoal,images,depths):
        process_images = self.process_image(images)
        process_depths = self.process_depth(depths)
        input_images = []
        for i in range(len(self.memory_queue)):
            if len(self.memory_queue[i]) < self.memory_size:
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])
                input_image = np.pad(input_image,((self.memory_size - input_image.shape[0],0),(0,0),(0,0),(0,0)))
            else:
                del self.memory_queue[i][0]
                self.memory_queue[i].append(process_images[i])
                input_image = np.array(self.memory_queue[i])

            input_images.append(input_image)
        input_image = np.array(input_images)
        input_depth = process_depths
        input_pointgoal = self.process_pointgoal(pointgoal)
        input_imagegoal = self.process_image(imagegoal)

        cv2.imwrite("input_image.jpg",np.concatenate(self.memory_queue[0],axis=0)*255)
        all_trajectory, all_values, good_trajectory, bad_trajectory = self.navi_former.predict_ip_action(input_pointgoal,input_imagegoal,input_image,input_depth)

        if all_values.max() < self.stop_threshold:
            good_trajectory[:,:,:,0] = good_trajectory[:,:,:,0] * 0.0
            good_trajectory[:,:,:,1] = np.sign(good_trajectory[:,:,:,1].mean())

        trajectory_mask = self.project_trajectory(images,all_trajectory,all_values)
        return good_trajectory[:,0], all_trajectory, all_values, trajectory_mask

    def _read_only_policy_inputs(self, images, depths):
        """Rebuild policy inputs from the FIFO without advancing it."""
        if len(images) != len(self.memory_queue):
            raise ValueError("observation batch differs from NavDP batch size")
        input_images = []
        for queue in self.memory_queue:
            if not queue:
                raise RuntimeError(
                    "mixed-goal resampling requires a prior policy step")
            input_image = np.asarray(queue)
            if input_image.shape[0] < self.memory_size:
                input_image = np.pad(
                    input_image,
                    ((self.memory_size - input_image.shape[0], 0),
                     (0, 0), (0, 0), (0, 0)),
                )
            input_images.append(input_image)
        input_images = np.asarray(input_images)
        input_depth = self.process_depth(depths)
        return input_images, input_depth

    def resample_point_image_goal(self, pointgoal, imagegoal, images, depths):
        """Sample mixed-goal candidates without advancing the RGB FIFO.

        A normal ImageGoal step for the current observation must run first.
        This lets a coverage residual compare a frontier-conditioned proposal
        against the native proposal while consuming the observation exactly
        once.
        """
        input_images, input_depth = self._read_only_policy_inputs(
            images, depths)
        input_pointgoal = self.process_pointgoal(pointgoal)
        input_imagegoal = self.process_image(imagegoal)
        all_trajectory, all_values, good_trajectory, bad_trajectory = (
            self.navi_former.predict_ip_action(
                input_pointgoal, input_imagegoal, input_images, input_depth))
        if all_values.max() < self.stop_threshold:
            good_trajectory[:, :, :, 0] = 0.0
            good_trajectory[:, :, :, 1] = np.sign(
                good_trajectory[:, :, :, 1].mean())
        trajectory_mask = self.project_trajectory(
            images, all_trajectory, all_values)
        return good_trajectory[:, 0], all_trajectory, all_values, trajectory_mask

    def score_imagegoal_trajectories(self, imagegoal, images, depths,
                                     trajectories, *, control_imagegoal=None,
                                     timesteps=None, noise_samples=1, seed=0):
        """Read-only ImageGoal/null contrast for already sampled candidates."""
        input_images, input_depth = self._read_only_policy_inputs(
            images, depths)
        input_imagegoal = self.process_image(imagegoal)
        input_control_goal = (
            self.process_image(control_imagegoal)
            if control_imagegoal is not None else None)
        return self.navi_former.score_imagegoal_trajectories(
            input_imagegoal,
            input_images,
            input_depth,
            trajectories,
            control_goal_image=input_control_goal,
            timesteps=timesteps,
            noise_samples=noise_samples,
            seed=seed,
        )
