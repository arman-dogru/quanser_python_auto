function summary = summarizeYoloPassiveLog(logFile)
%SUMMARIZEYOLOPASSIVELOG Summarize a Phase 5 passive YOLO flight log.
arguments
    logFile (1,1) string
end
assert(isfile(logFile), 'Log not found: %s', logFile);
data = readtable(logFile);
required = {'timestamp','detected','centerX','centerY','confidence', ...
    'frame','inference_ms','observation_age_ms'};
assert(all(ismember(required, data.Properties.VariableNames)), ...
    'The selected file is not a Phase 5 passive YOLO log.');

detected = data.detected == 1;
summary.frames = height(data);
summary.detections = nnz(detected);
summary.detectionRate = nnz(detected) / max(height(data), 1);
summary.durationSeconds = max(data.timestamp) - min(data.timestamp);
summary.meanInferenceMs = mean(data.inference_ms, 'omitnan');
summary.maxObservationAgeMs = max(data.observation_age_ms);
if any(detected)
    summary.meanConfidence = mean(data.confidence(detected), 'omitnan');
    summary.minimumConfidence = min(data.confidence(detected));
    summary.centerXRange = [min(data.centerX(detected)), max(data.centerX(detected))];
    summary.centerYRange = [min(data.centerY(detected)), max(data.centerY(detected))];
else
    summary.meanConfidence = NaN;
    summary.minimumConfidence = NaN;
    summary.centerXRange = [NaN NaN];
    summary.centerYRange = [NaN NaN];
end
disp(summary)
end
