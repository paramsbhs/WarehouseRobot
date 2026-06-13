%% Benchmark: prioritized planning vs Conflict-Based Search
% Runs both planners on identical random instances for increasing robot
% counts and compares success rate, sum of costs, makespan, and planning
% time. CBS gets slower at higher robot counts; that trade-off is the
% point of the comparison.

clear; clc;

map = createWarehouseMap(1);
inflate(map, 0.6);
occ = occupancyMatrix(map);

[freeR, freeC] = find(~occ);
freeCells = [freeR, freeC];

robotCounts = [2 4 6 8];
nTrials = 5;
maxT = 300;
rng(1);

nC = numel(robotCounts);
sucP = zeros(nC, 1); sucC = zeros(nC, 1);
costP = nan(nC, nTrials); costC = nan(nC, nTrials);
makeP = nan(nC, nTrials); makeC = nan(nC, nTrials);
timeP = nan(nC, nTrials); timeC = nan(nC, nTrials);

for k = 1:nC
    n = robotCounts(k);
    fprintf("=== %d robots ===\n", n);
    for trial = 1:nTrials
        idx = randperm(size(freeCells, 1), 2 * n);
        starts = freeCells(idx(1:n), :);
        goals  = freeCells(idx(n+1:end), :);

        tic;
        [~, infoP] = planMAPF_Prioritized(occ, starts, goals, maxT);
        timeP(k, trial) = toc;

        tic;
        [~, infoC] = planMAPF_CBS(occ, starts, goals, maxT);
        timeC(k, trial) = toc;

        if infoP.success
            sucP(k) = sucP(k) + 1;
            costP(k, trial) = infoP.cost;
            makeP(k, trial) = infoP.makespan;
        end
        if infoC.success
            sucC(k) = sucC(k) + 1;
            costC(k, trial) = infoC.cost;
            makeC(k, trial) = infoC.makespan;
        end
        fprintf("  trial %d: prioritized cost=%g (%.2fs)  CBS cost=%g (%.2fs, %d expansions)\n", ...
            trial, infoP.cost, timeP(k, trial), ...
            infoC.cost, timeC(k, trial), infoC.expansions);
    end
end

%% Summary table
summary = table(robotCounts(:), ...
    sucP / nTrials, sucC / nTrials, ...
    mean(costP, 2, "omitnan"), mean(costC, 2, "omitnan"), ...
    mean(makeP, 2, "omitnan"), mean(makeC, 2, "omitnan"), ...
    mean(timeP, 2, "omitnan"), mean(timeC, 2, "omitnan"), ...
    VariableNames = ["Robots", ...
        "SuccessPrio", "SuccessCBS", ...
        "CostPrio", "CostCBS", ...
        "MakespanPrio", "MakespanCBS", ...
        "TimePrio_s", "TimeCBS_s"]);
disp(summary);

%% Plots
figure(Name = "Prioritized vs CBS");
subplot(1, 2, 1);
plot(robotCounts, mean(costP, 2, "omitnan"), "o-", ...
     robotCounts, mean(costC, 2, "omitnan"), "s-", LineWidth = 1.5);
xlabel("Number of robots"); ylabel("Mean sum of costs (timesteps)");
legend("Prioritized", "CBS (optimal)", Location = "northwest");
grid on; title("Solution quality");

subplot(1, 2, 2);
semilogy(robotCounts, mean(timeP, 2, "omitnan"), "o-", ...
         robotCounts, mean(timeC, 2, "omitnan"), "s-", LineWidth = 1.5);
xlabel("Number of robots"); ylabel("Mean planning time (s)");
legend("Prioritized", "CBS", Location = "northwest");
grid on; title("Planning time");

%% Animate one CBS solution
n = 4;
idx = randperm(size(freeCells, 1), 2 * n);
[paths, info] = planMAPF_CBS(occ, freeCells(idx(1:n), :), ...
    freeCells(idx(n+1:end), :), maxT);
if info.success
    animatePaths(map, paths, sprintf("CBS solution, cost = %d", info.cost));
end
