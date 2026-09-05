import { PanelExtensionContext, RenderState } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";

type ActionId = "capture-goal" | "start-survey" | "stop-survey" | "revisit" | "stop-navigation";
type Tone = "capture" | "primary" | "neutral" | "revisit" | "danger";

type Action = {
  id: ActionId;
  label: string;
  pendingLabel: string;
  serviceNames: readonly string[];
  title: string;
  tone: Tone;
};

type Notice = {
  message: string;
  tone: "muted" | "success" | "error";
};

type WorkflowStatus = {
  active: boolean;
  allowed_actions?: readonly string[];
  capture_active?: boolean;
  dataset_id?: string;
  detail: string;
  episode_id?: string;
  episode_state?: string;
  goal_captured_utc?: string;
  state: string;
};

const WORKFLOW_TOPIC = "/navdp/operator/revisit_workflow";

const ACTIONS: readonly Action[] = [
  {
    id: "capture-goal",
    label: "CAPTURE GOAL",
    pendingLabel: "CAPTURING…",
    serviceNames: ["/memnav_operator/capture_goal"],
    title: "Create a new Episode and freeze the current aligned RGB-D frame",
    tone: "capture",
  },
  {
    id: "start-survey",
    label: "START SURVEY",
    pendingLabel: "STARTING…",
    serviceNames: ["/memnav_operator/start_survey"],
    title: "Automatically prepare the Dataset and start RGB Survey recording",
    tone: "primary",
  },
  {
    id: "stop-survey",
    label: "STOP SURVEY",
    pendingLabel: "STOPPING…",
    serviceNames: ["/memnav_operator/stop_survey"],
    title: "Stop, validate, persist and seal the Survey Dataset",
    tone: "neutral",
  },
  {
    id: "revisit",
    label: "REVISIT",
    pendingLabel: "STARTING…",
    serviceNames: ["/memnav_operator/start_revisit"],
    title: "Prepare the sealed Survey stack and begin supervised Revisit",
    tone: "revisit",
  },
  {
    id: "stop-navigation",
    label: "STOP",
    pendingLabel: "STOPPING…",
    serviceNames: ["/memnav_operator/operator_stop"],
    title: "Stop motion and save recording; this does not mark the experiment as failed",
    tone: "danger",
  },
] as const;

function responseNotice(action: Action, response: unknown): Notice {
  let message = "";
  if (typeof response === "object" && response != undefined) {
    const result = response as { message?: unknown; success?: unknown };
    if (typeof result.message === "string") {
      const raw = result.message.trim();
      try {
        const payload = JSON.parse(raw) as {
          detail?: unknown;
          message?: unknown;
          operator_summary?: unknown;
        };
        const readable = payload.operator_summary ?? payload.detail ?? payload.message;
        message = typeof readable === "string" ? readable.trim() : "";
      } catch {
        message = raw;
      }
    }
    if (result.success === false) {
      return {
        message: message || `${action.label} was rejected`,
        tone: "error",
      };
    }
  }
  return { message: message || `${action.label} complete`, tone: "success" };
}

function errorMessage(error: unknown): string {
  if (error instanceof Error && error.message.trim()) {
    return error.message;
  }
  return "Service call failed";
}

function workflowFromFrame(
  frame: readonly { readonly message: unknown; readonly topic: string }[] | undefined,
): WorkflowStatus | undefined {
  if (frame == undefined) {
    return undefined;
  }
  for (let index = frame.length - 1; index >= 0; index -= 1) {
    const event = frame[index];
    if (event?.topic !== WORKFLOW_TOPIC || typeof event.message !== "object") {
      continue;
    }
    const data = (event.message as { data?: unknown }).data;
    if (typeof data !== "string") {
      continue;
    }
    try {
      const payload = JSON.parse(data) as Partial<WorkflowStatus>;
      if (
        typeof payload.active === "boolean" &&
        typeof payload.detail === "string" &&
        typeof payload.state === "string"
      ) {
        return {
          active: payload.active,
          allowed_actions: Array.isArray(payload.allowed_actions)
            ? payload.allowed_actions.filter((item): item is string => typeof item === "string")
            : undefined,
          capture_active:
            typeof payload.capture_active === "boolean" ? payload.capture_active : undefined,
          dataset_id: typeof payload.dataset_id === "string" ? payload.dataset_id : undefined,
          detail: payload.detail,
          episode_id: typeof payload.episode_id === "string" ? payload.episode_id : undefined,
          episode_state:
            typeof payload.episode_state === "string" ? payload.episode_state : undefined,
          goal_captured_utc:
            typeof payload.goal_captured_utc === "string" ? payload.goal_captured_utc : undefined,
          state: payload.state,
        };
      }
    } catch {
      // Ignore malformed status and keep the last valid workflow state.
    }
  }
  return undefined;
}

function workflowNotice(status: WorkflowStatus): Notice {
  if (status.state === "complete") {
    return { message: status.detail, tone: "success" };
  }
  if (status.state === "blocked" || status.state === "failed") {
    return { message: status.detail, tone: "error" };
  }
  return { message: status.detail, tone: "muted" };
}

function OperatorControls({ context }: { context: PanelExtensionContext }): ReactElement {
  const [colorScheme, setColorScheme] = useState<RenderState["colorScheme"]>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [pending, setPending] = useState<ReadonlySet<ActionId>>(() => new Set());
  const [notice, setNotice] = useState<Notice>({ message: "Ready", tone: "muted" });
  const [workflow, setWorkflow] = useState<WorkflowStatus>();

  useLayoutEffect(() => {
    context.watch("colorScheme");
    context.watch("currentFrame");
    context.subscribe([{ topic: WORKFLOW_TOPIC }]);
    context.onRender = (renderState, done) => {
      setColorScheme(renderState.colorScheme);
      const nextWorkflow = workflowFromFrame(renderState.currentFrame);
      if (nextWorkflow != undefined) {
        setWorkflow(nextWorkflow);
        setNotice(workflowNotice(nextWorkflow));
      }
      setRenderDone(() => done);
    };
    return () => {
      context.subscribe([]);
    };
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const callAction = useCallback(
    async (action: Action): Promise<void> => {
      if (context.callService == undefined) {
        setNotice({ message: "Services unavailable for this connection", tone: "error" });
        return;
      }
      const callService = async (serviceName: string): Promise<unknown> =>
        await (context.callService?.(serviceName, {}) ??
          Promise.reject(new Error("Services unavailable")));

      setPending((current) => new Set(current).add(action.id));
      setNotice({
        message: `${action.pendingLabel.replace("…", "").toLowerCase()}…`,
        tone: "muted",
      });
      try {
        const results = await Promise.allSettled(
          action.serviceNames.map(async (serviceName) => await callService(serviceName)),
        );
        const accepted = results.find(
          (result) =>
            result.status === "fulfilled" &&
            !(
              typeof result.value === "object" &&
              result.value != undefined &&
              (result.value as { success?: unknown }).success === false
            ),
        );
        if (accepted?.status === "fulfilled") {
          setNotice(responseNotice(action, accepted.value));
        } else {
          const reasons = results.map((result) =>
            result.status === "rejected"
              ? errorMessage(result.reason)
              : responseNotice(action, result.value).message,
          );
          setNotice({ message: reasons.join(" · "), tone: "error" });
        }
      } catch (error) {
        setNotice({ message: errorMessage(error), tone: "error" });
      } finally {
        setPending((current) => {
          const next = new Set(current);
          next.delete(action.id);
          return next;
        });
      }
    },
    [context],
  );

  const activateAction = useCallback(
    (action: Action): void => {
      void callAction(action);
    },
    [callAction],
  );

  const servicesAvailable = context.callService != undefined;
  const status = servicesAvailable
    ? notice
    : { message: "Services unavailable for this connection", tone: "error" as const };
  const anyPending = pending.size > 0;
  const episodeLabel = workflow?.episode_id?.replace(/^episode_/, "") ?? "NO EPISODE";
  const stageLabel = (workflow?.episode_state ?? workflow?.state ?? "idle")
    .replace(/_/g, " ")
    .toUpperCase();

  return (
    <div className="operator-controls" data-color-scheme={colorScheme ?? "dark"}>
      <div className="episode-strip">
        <span className="episode-id" title={workflow?.episode_id ?? "No active Episode"}>
          {episodeLabel}
        </span>
        <span className="episode-stage">{stageLabel}</span>
        {workflow?.capture_active === true && <span className="recording-badge">REC RGB-D</span>}
      </div>
      <div className="control-row">
        {ACTIONS.map((action) => {
          const isPending = pending.has(action.id);
          const disabled =
            !servicesAvailable ||
            isPending ||
            (workflow?.allowed_actions != undefined
              ? !workflow.allowed_actions.includes(action.id)
              : action.id !== "stop-navigation" && (anyPending || workflow?.active === true));
          return (
            <button
              key={action.id}
              aria-busy={isPending}
              className={`control-button control-button--${action.tone}`}
              disabled={disabled}
              title={action.title}
              type="button"
              onClick={() => {
                activateAction(action);
              }}
            >
              <span>{isPending ? action.pendingLabel : action.label}</span>
            </button>
          );
        })}
      </div>
      <div
        aria-live="polite"
        className="control-status"
        data-tone={status.tone}
        title={status.message}
      >
        <span className="status-dot" />
        <span className="status-copy">{status.message}</span>
      </div>
    </div>
  );
}

export function initOperatorControls(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OperatorControls context={context} />);
  return () => {
    root.unmount();
  };
}
