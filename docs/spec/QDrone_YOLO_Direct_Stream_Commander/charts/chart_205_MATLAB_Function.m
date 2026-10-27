function [stream_connection_loss , data_loss] = fcn(time, state, error, ms_time)

persistent ms_time_prev
    if isempty(ms_time_prev)
        ms_time_prev = 0;
    end

stream_connection_loss = 0;
data_loss = 0;
if time > 3
    if ((state ~= 2)&&(error < 0))
        % stream connection is not established correctly
        stream_connection_loss = 1;
    end

    if (ms_time == ms_time_prev)&&(ms_time > 0)
        % data is stale
        data_loss = 1;
    end
end

ms_time_prev = ms_time;
