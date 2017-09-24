
dir_path = "optim_param/"

# copied right out of RVO2DCOnfig.py
# remove the id param though
rvo_parameters = """rvo_neighbor_distance,
rvo_time_horizon,
rvo_max_speed,
rvo_time_horizon_obstacles,
rvo_max_neighbors"""
rvo_parameters = rvo_parameters.translate(None,'\n')
print rvo_parameters
rvo_parameters_list = rvo_parameters.split(',')
print rvo_parameters_list


for param in rvo_parameters_list:
    output_quiery_string = """select sum(test_status) as "scenarios_passed",
    sum((pp.test_status + 0.0 ))/count(*) as "c(Av)",
    1 - sum((pp.test_status + 0.0 ))/count(*) as "f(Av)",
    1 - sum(((1.0/300)/pp.num_agents)/(pp.average_time_per_call/pp.num_agents))/count(*) as "eff(Av)",
    1 - (SUM( CASE WHEN pp.test_status = 1 THEN pp.optimal_path_length / pp.total_distance_traveled ELSE 0 END)/count(*)) as "distance(Av)",
    1 - (SUM( CASE WHEN pp.test_status = 1 THEN pp.ple_energy_optimal / pp.ple_energy ELSE 0 END)/count(*)) as "ple(Av)",
    1 - (SUM( CASE WHEN pp.test_status = 1 THEN pp.optimal_time / pp.total_time_enabled ELSE 0 END)/count(*)) as "time(Av)",
    sum(CASE WHEN pp.test_status = 1 THEN 0 ELSE 1 END) as "scenarios failed",
    sum(CASE WHEN pp.test_status = 1 THEN 0 ELSE 1.0 END)/count(*) as "scenarios failed percent",
    count(*), 
    c.config_id, 
    pp.scenario_group, 
    rc.""" + param + """,
    AVG(pp.total_ticks_accumulated/pp.frames) as "average_total_ticks for frames",
    AVG(pp.total_ticks_accumulated) as "average_total_ticks",
    AVG(pp.average_time_per_call) as "average_total_time_per_call",
    AVG(pp.total_time_of_all_calls) as "average_total_time_all_calls",
    AVG(pp.number_of_times_executed) as "average_total_times_called",
    AVG(pp.frames) as "average_number_of_frames"
from rvo2d_par_opt_"""+ param+"""_view pp, scenario s, config c, rvo2d_ai_config rc 
where 
    pp.scenario_group = s.scenario_id and 
    s.config_id = c.config_id and 
    c.config_id = rc.rvo2d_ai_config_id 
    group by 
        scenario_group, c.config_id, rc.""" + param + """ 
    order by 
        rc.""" + param + """;"""
        
    # print output_quiery_string
    output_param_file = open(dir_path+param+".sql", "w")
    output_param_file.write(output_quiery_string)
    output_param_file.close()
        

        