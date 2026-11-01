"""Release-gated retreat and empty folding. Inputs are measured control signals."""

import math


class ReturnSequence:
    def __init__(self, parameters, experiment, approach):
        self.p, self.e, self.approach = parameters, experiment, approach
        self.enabled = experiment.get('return_after_release', False)
        if self.enabled and not approach['enabled']:
            raise ValueError('Return sequence requires a folded-start mission')
        self.state = 'inactive'
        self.started = self.changed = None
        self.start_x = self.retreat_from = None
        self.retreat_started = self.fold_started = None
        self.failure = None
        self.successful = self.done = False
        self.events = []
        self.window = []
        self.opened = approach['open_pose']
        self.closed = approach['raised_closed_pose']
        self.folded = approach['folded_pose']
        self.fold_duration = max(abs(self.closed[j]-self.folded[j])/parameters['dynamic_arm_rate_rad_s']
                                 for j in ['shoulder','elbow','wrist'])
        self.home_error_before_folding = None

    def change(self, state, now):
        self.state, self.changed = state, now
        self.window = []
        self.events.append({'state':state,'time_s':now})

    def stable(self, now, condition, seconds=.5):
        self.window.append((now,bool(condition)))
        self.window = [r for r in self.window if r[0]>=now-seconds]
        return (self.window[-1][0]-self.window[0][0]>=seconds-.02 and all(r[1] for r in self.window))

    def update(self, now, trigger, x, velocity, release_ready, pose_error, pitch_error, preflight_rejected):
        if not self.enabled or now<trigger:
            return None
        if self.state=='inactive':
            self.started, self.start_x = now, x
            self.change('skipped_preflight' if preflight_rejected else 'hold_control' if self.e.get('test_hold_after_release',False) else 'await_release',now)
        if self.state=='skipped_preflight':
            self.done = True
            return None
        if self.state=='hold_control' and now-self.started>=45.0:
            self.done = True
        if self.state=='await_release':
            confirmed = release_ready and not self.e.get('test_block_release_confirmation',False)
            if self.stable(now, confirmed and pose_error(self.opened)<.01):
                self.retreat_started, self.retreat_from = now, x
                self.change('retreat',now)
            elif now-self.changed>=6.0:
                self.failure = 'release_unconfirmed'
                self.change('blocked_release',now)
        if self.state=='retreat':
            elapsed = now-self.retreat_started
            if self.stable(now, elapsed>=6 and abs(x)<.0005 and abs(velocity)<.003 and pose_error(self.opened)<.01):
                self.change('close_empty',now)
            elif elapsed>=self.e.get('return_retreat_timeout_s',40.0):
                self.failure = 'retreat_unsettled'
                self.change('blocked_retreat',now)
        if self.state=='close_empty':
            ready = (now-self.changed>=2.5 and pose_error(self.closed)<.01 and abs(x)<.002 and abs(velocity)<.003)
            if self.stable(now,ready):
                self.fold_started, self.home_error_before_folding = now, abs(x)
                self.change('folding',now)
            elif now-self.changed>=12.0:
                self.failure = 'empty_claw_tracking_timeout'
                self.change('failed_claw',now)
        if self.state=='folding':
            ready = (now-self.fold_started>=self.fold_duration+1 and pose_error(self.folded)<.01
                     and abs(x)<.005 and abs(velocity)<.003 and abs(pitch_error)<math.radians(1))
            if self.stable(now,ready,2.0):
                self.successful = self.done = True
                self.change('returned_folded',now)
            elif now-self.fold_started>=40.0:
                self.failure = 'fold_tracking_timeout'
                self.change('failed_fold',now)
        if self.state in ['blocked_release','failed_claw','failed_fold'] and now-self.changed>=2:
            self.done = True
        if self.state=='blocked_retreat' and now-self.changed>=8:
            self.done = True
        if self.state in ['await_release','blocked_release','hold_control']:
            return 'return_'+self.state,self.opened,self.start_x,0.0
        if self.state=='retreat':
            u = max(0.0,min(1.0,(now-self.retreat_started)/6.0))
            distance = -self.retreat_from
            return 'return_retreat',self.opened,self.retreat_from+distance*(3*u*u-2*u*u*u),distance*u*(1-u)
        if self.state in ['close_empty','failed_claw']:
            return 'return_'+self.state,self.closed,0.0,0.0
        if self.state=='blocked_retreat':
            return 'return_blocked_retreat',self.opened,0.0,0.0
        return 'return_'+self.state,self.folded,0.0,0.0

    def report(self):
        return {key:getattr(self,key) for key in ['enabled','state','started','start_x','retreat_started','fold_started',
                 'failure','successful','done','home_error_before_folding','events']}
