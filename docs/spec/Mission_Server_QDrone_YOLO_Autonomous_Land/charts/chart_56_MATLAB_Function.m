function stream_connection_loss  = fcn(time, state, error)

stream_connection_loss = 0;
if time > 1
    if ~((state == 2)&&(error >= 0))
        % stream connection is not established correctly
        stream_connection_loss = 1;
    end
end
