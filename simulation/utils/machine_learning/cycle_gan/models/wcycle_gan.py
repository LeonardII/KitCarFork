from typing import List, Tuple

import torch
from torch import Tensor, nn

from simulation.utils.machine_learning.models.helper import set_requires_grad

from .base_model import BaseModel, CycleGANNetworks
from .cycle_gan_stats import CycleGANStats


class WassersteinCycleGANModel(BaseModel):
    """This class implements the CycleGAN model, for learning image-to-image translation
    without paired data.

    By default, it uses a '--netg resnet_9blocks' ResNet generator,
    a '--netd basic' discriminator (PatchGAN introduced by pix2pix),
    and a least-square GANs objective ('--gan_mode lsgan').

    CycleGAN paper: https://arxiv.org/pdf/1703.10593.pdf
    """

    def __init__(
        self,
        netg_a_to_b: nn.Module,
        netg_b_to_a: nn.Module,
        netd_a: nn.Module = None,
        netd_b: nn.Module = None,
        is_train: bool = True,
        beta1: float = 0.5,
        lr: float = 0.0002,
        lr_policy: str = "linear",
        lambda_idt_a: int = 10,
        lambda_idt_b: int = 10,
        lambda_cycle: float = 0.5,
        optimizer_type: str = "rms_prop",
        is_l1: bool = False,
        wgan_n_critic: int = 5,
        wgan_initial_n_critic: int = 5,
        wgan_clip_lower=-0.01,
        wgan_clip_upper=0.01,
    ):
        """Initialize the CycleGAN class.

        Args:
            is_train: enable or disable training mode
            beta1: momentum term of adam
            lr: initial learning rate for adam
            lr_policy: linear #learning rate policy. [linear | step | plateau | cosine]
            lambda_idt_a: weight for loss of domain A
            lambda_idt_b: weight for loss of domain B
            lambda_cycle: weight for loss identity
            is_l1: Decide whether to use l1 loss or l2 loss as cycle
                and identity loss functions
        """
        self.wgan_initial_n_critic = wgan_initial_n_critic
        self.clips = (wgan_clip_lower, wgan_clip_upper)

        self.wgan_n_critic = wgan_n_critic

        self.networks = CycleGANNetworks(netg_a_to_b, netg_b_to_a, netd_a, netd_b)

        super().__init__(
            netg_a_to_b,
            netg_b_to_a,
            netd_a,
            netd_b,
            is_train,
            lambda_cycle,
            lambda_idt_a,
            lambda_idt_b,
            is_l1,
            optimizer_type,
            lr_policy,
            beta1,
            lr,
        )

    def update_critic_a(
        self,
        batch_a: Tensor,
        batch_b: Tensor,
        clip_bounds: Tuple[float, float] = None,
    ):
        set_requires_grad([self.networks.d_a], requires_grad=True)
        return self.networks.d_a.perform_optimization_step(
            self.networks.g_a_to_b,
            self.optimizer_d,
            batch_a,
            batch_b,
            clip_bounds,
        )

    def update_critic_b(
        self,
        batch_a: Tensor,
        batch_b: Tensor,
        clip_bounds: Tuple[float, float] = None,
    ):
        set_requires_grad([self.networks.d_b], requires_grad=True)
        return self.networks.d_b.perform_optimization_step(
            self.networks.g_b_to_a,
            self.optimizer_d,
            batch_b,
            batch_a,
            clip_bounds,
        )

    def update_generators(self, batch_a: Tensor, batch_b: Tensor):
        """"""
        real_a = batch_a
        real_b = batch_b

        self.optimizer_g.zero_grad()  # set G_A and G_B's gradients to zero
        # G_A and G_B
        set_requires_grad(
            [self.networks.d_a, self.networks.d_b], False
        )  # Ds require no gradients when optimizing Gs
        set_requires_grad([self.networks.g_a_to_b, self.networks.g_b_to_a], True)

        fake_a = self.networks.g_b_to_a(real_b)
        loss_g_b_to_a = -torch.mean(self.networks.d_a(fake_a))

        fake_b = self.networks.g_a_to_b(real_a)
        loss_g_a_to_b = -torch.mean(self.networks.d_b(fake_b))

        # Identity loss
        idt_a = self.networks.g_b_to_a(real_a)
        idt_b = self.networks.g_a_to_b(real_b)
        loss_idt_a = self.criterionIdt(idt_a, real_a) * self.lambda_idt_a
        loss_idt_b = self.criterionIdt(idt_b, real_b) * self.lambda_idt_b

        rec_a = self.networks.g_a_to_b(fake_a)
        rec_b = self.networks.g_b_to_a(fake_b)

        # Forward cycle loss
        loss_cycle_a = self.criterionCycle(rec_a, real_a) * self.lambda_cycle
        # Backward cycle loss
        loss_cycle_b = self.criterionCycle(rec_b, real_b) * self.lambda_cycle

        # combined loss and calculate gradients
        loss_g = (
            loss_g_a_to_b
            + loss_g_b_to_a
            + loss_cycle_a
            + loss_cycle_b
            + loss_idt_a
            + loss_idt_b
        )
        loss_g.backward()

        self.optimizer_g.step()  # update G_A and G_B's weights

        return CycleGANStats(
            real_a,
            real_b,
            fake_a,
            fake_b,
            rec_a,
            rec_b,
            idt_a,
            idt_b,
            loss_g_a_to_b.item(),
            loss_g_b_to_a.item(),
            loss_idt_a.item(),
            loss_idt_b.item(),
            loss_cycle_a.item(),
            loss_cycle_b.item(),
        )

    def pre_training(self, critic_batches):
        # Update critic
        for batch_a, batch_b in critic_batches:
            self.update_critic_a(batch_a, batch_b, self.clips)
            self.update_critic_b(batch_a, batch_b, self.clips)

    def do_iteration(
        self,
        batch_a: torch.Tensor,
        batch_b: torch.Tensor,
        critic_batches: List[Tuple[torch.Tensor, torch.Tensor]],
    ):
        # Update critic
        for batch_a_d, batch_b_d in critic_batches:
            distance_a = self.update_critic_a(batch_a_d, batch_b_d, self.clips)
            distance_b = self.update_critic_b(batch_a_d, batch_b_d, self.clips)

        update_stats: CycleGANStats = self.update_generators(batch_a, batch_b)
        update_stats.w_distance_a = distance_a
        update_stats.w_distance_b = distance_b

        return update_stats
