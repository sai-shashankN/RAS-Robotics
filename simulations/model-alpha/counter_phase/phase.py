"""Set-valued output-event phases under explicit manufactured schedule assumptions."""
def integer(v):return type(v) is int
class CounterPhaseModel:
    def __init__(self,period_ticks=128,modulus_ticks=1<<24,max_gap_ticks=4096,origin_raw_tick=0,epoch=1):
        if not all(integer(x) for x in [period_ticks,modulus_ticks,max_gap_ticks,origin_raw_tick,epoch]):raise ValueError('integer model parameters')
        if not(1<=period_ticks<=4096 and modulus_ticks>2*period_ticks and 0<max_gap_ticks<modulus_ticks and 0<=origin_raw_tick<modulus_ticks and epoch>0):raise ValueError('bounded manufactured clock parameters')
        self.period=period_ticks;self.modulus=modulus_ticks;self.max_gap=max_gap_ticks;self.epoch=epoch;self.raw=origin_raw_tick;self.elapsed=0;self.total=0;self.sequence=0;self.phases=set(range(1,period_ticks+1));self.fault=None
    def observe(self,raw_read_tick,total_output_frames,observation_sequence,epoch,minimum_elapsed_ticks=0,maximum_elapsed_ticks=None,provenance='manufactured'):
        if self.fault is None:
            try:
                maximum=self.max_gap if maximum_elapsed_ticks is None else maximum_elapsed_ticks
                if provenance!='manufactured':raise ValueError('manufactured schedule evidence only')
                if not all(integer(v) for v in [raw_read_tick,total_output_frames,observation_sequence,epoch,minimum_elapsed_ticks,maximum]):raise ValueError('integer counter/count/sequence/interval')
                if epoch!=self.epoch:raise ValueError('epoch changed;new conditional model required')
                if observation_sequence!=self.sequence+1:raise ValueError('observation sequence gap/duplicate')
                if not(0<=raw_read_tick<self.modulus and total_output_frames>=self.total and 0<=minimum_elapsed_ticks<=maximum<=self.max_gap):raise ValueError('counter/count/declared gap bounds')
                delta=(raw_read_tick-self.raw)%self.modulus
                if not(minimum_elapsed_ticks<=delta<=maximum):raise ValueError('modular delta outside declared sub-wrap gap')
                elapsed=self.elapsed+delta
                candidates={p for p in self.phases if (0 if elapsed<p else (elapsed-p)//self.period+1)==total_output_frames}
                if not candidates:raise ValueError('no phase satisfies complete cumulative count')
                self.phases=candidates;self.raw=raw_read_tick;self.elapsed=elapsed;self.total=total_output_frames;self.sequence=observation_sequence
            except (ValueError,TypeError) as error:self.fault=type(error).__name__+':'+str(error)
        phases=sorted(self.phases) if self.fault is None else [];bounds=ages=None
        if phases and self.total:
            ticks=[p+(self.total-1)*self.period for p in phases];bounds=[min(ticks),max(ticks)];ages=[self.elapsed-bounds[1],self.elapsed-bounds[0]]
        return dict(status='latched_unavailable' if self.fault else 'conditional_output_event_phase_set',fault=self.fault,model_epoch=self.epoch,conditional_read_elapsed_ticks=self.elapsed if not self.fault else None,cumulative_output_frames=self.total if not self.fault else None,candidate_phases_ticks=phases,conditional_last_output_elapsed_tick_bounds=bounds,conditional_last_output_age_tick_bounds=ages,last_accepted_elapsed_ticks=self.elapsed,last_accepted_sequence=self.sequence,model_period_ticks=self.period,sub_wrap_gap_is_declared_assumption=True,complete_no_loss_fixed_period_is_declared_assumption=True,acquisition_s=None,output_event_phase_physically_qualified=False,conversion_freshness_proven=False,sample_sequence_proven=False,host_device_clock_mapping_qualified=False,hardware_driver_executed=False,consumer_reset_authorized=False,usable_for_feedback=False)
