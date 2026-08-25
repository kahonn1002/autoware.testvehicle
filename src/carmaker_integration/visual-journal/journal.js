(() => {
  "use strict";

  const data = window.PROCESS_JOURNAL;
  if (!data) {
    document.body.innerHTML = "<p>Project data could not be loaded.</p>";
    return;
  }

  const escapeHtml = (value) =>
    String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");

  const project = data.project;
  document.title = `${project.title} · Process Journal`;
  document.getElementById("projectEyebrow").textContent = project.eyebrow;
  document.getElementById("projectTitle").textContent = project.title;
  document.getElementById("projectSummary").textContent = project.summary;
  document.getElementById("footerProject").textContent =
    `${project.status} · ${project.date}`;

  document.getElementById("statusRow").innerHTML = [
    project.status,
    `TestRun: ${project.testRun}`,
    project.date
  ].map((item) => `<span class="status-chip">${escapeHtml(item)}</span>`).join("");

  document.getElementById("instrumentGrid").innerHTML = project.facts
    .map((fact) => `
      <div class="instrument-stat">
        <strong>${escapeHtml(fact.value)}</strong>
        <span>${escapeHtml(fact.label)}</span>
      </div>
    `).join("");

  document.getElementById("architectureFlow").innerHTML = data.architecture
    .map((node) => `
      <article class="architecture-node" tabindex="0" data-type="${escapeHtml(node.type)}">
        <span class="node-label">${escapeHtml(node.label)}</span>
        <h3>${escapeHtml(node.title)}</h3>
        <p>${escapeHtml(node.detail)}</p>
      </article>
    `).join("");

  const debugChain = (bug) => `
    <div class="debug-chain">
      <div class="debug-cell symptom">
        <span class="debug-label">PROBLEM</span>
        <p>${escapeHtml(bug.symptom)}</p>
      </div>
      <div class="debug-cell diagnosis">
        <span class="debug-label">ROOT CAUSE</span>
        <p>${escapeHtml(bug.diagnosis)}</p>
      </div>
      <div class="debug-cell fix">
        <span class="debug-label">FIX</span>
        <p>${escapeHtml(bug.fix)}</p>
      </div>
      <div class="debug-cell result">
        <span class="debug-label">RESULT</span>
        <p>${escapeHtml(bug.result)}</p>
      </div>
    </div>
  `;

  const implementationDetails = (stage) => {
    const renderSnapshots = (snapshots = []) => snapshots.map((snapshot) => {
      const startLine = Number(snapshot.startLine || 1);
      const lines = String(snapshot.content || "").split("\n").map((line, offset) => `
        <span class="snapshot-line">
          <i>${startLine + offset}</i>
          <b>${escapeHtml(line) || " "}</b>
        </span>
      `).join("");
      return `
        <details class="code-snapshot">
          <summary>
            <span>VIEW CODE SNAPSHOT</span>
            <small>${escapeHtml(snapshot.path)}:${startLine}</small>
          </summary>
          <div class="snapshot-head">
            <code>${escapeHtml(snapshot.path)}</code>
            <span>${escapeHtml(snapshot.language || "text")}</span>
          </div>
          <pre aria-label="Code excerpt from ${escapeHtml(snapshot.path)}"><code>${lines}</code></pre>
        </details>
      `;
    }).join("");

    const steps = (stage.implementation || []).map((step, index) => `
      <li>
        <span class="recipe-number">${String(index + 1).padStart(2, "0")}</span>
        <div>
          <strong>${escapeHtml(step.title)}</strong>
          <p>${escapeHtml(step.detail)}</p>
          ${renderSnapshots(step.code)}
        </div>
      </li>
    `).join("");
    const files = (stage.files || []).map((file) =>
      `<code class="file-chip">${escapeHtml(file)}</code>`).join("");
    const config = (stage.config || []).map((entry) => `
      <div class="config-entry">
        <span>${escapeHtml(entry.key)}</span>
        <strong>${escapeHtml(entry.value)}</strong>
      </div>
    `).join("");
    const commands = (stage.commands || []).map((command) =>
      `<pre><code>${escapeHtml(command)}</code></pre>`).join("");
    const verification = (stage.verify || []).map((item) =>
      `<li>${escapeHtml(item)}</li>`).join("");

    return `
      <section class="implementation-recipe">
        <div class="recipe-head">
          <span>IMPLEMENTATION RECIPE</span>
          <p>Concrete repository, file, configuration, and verification work.</p>
        </div>
        ${steps ? `<ol class="recipe-steps">${steps}</ol>` : ""}
        ${config ? `
          <div class="detail-block">
            <span class="detail-label">KEY CONFIGURATION</span>
            <div class="config-grid">${config}</div>
          </div>` : ""}
        ${files ? `
          <div class="detail-block">
            <span class="detail-label">FILES</span>
            <div class="file-list">${files}</div>
          </div>` : ""}
        ${commands ? `
          <div class="detail-block">
            <span class="detail-label">COMMANDS</span>
            <div class="command-list">${commands}</div>
          </div>` : ""}
        ${verification ? `
          <div class="detail-block verification-block">
            <span class="detail-label">VERIFY BEFORE CONTINUING</span>
            <ul>${verification}</ul>
          </div>` : ""}
      </section>
    `;
  };

  document.getElementById("timeline").innerHTML = data.stages.map((stage) => {
    const hasBugs = stage.bugs && stage.bugs.length > 0;
    const bugCount = hasBugs ? stage.bugs.length : 0;
    const result = !hasBugs && stage.result
      ? `<div class="stage-result"><span>RESULT</span><p>${escapeHtml(stage.result)}</p></div>`
      : "";
    return `
      <article class="stage ${hasBugs ? "has-bugs" : ""}"
        data-bugs="${bugCount}" data-critical="${Boolean(stage.critical)}" id="stage-${escapeHtml(stage.id)}">
        <div class="stage-index">${escapeHtml(stage.number)}</div>
        <div class="stage-card">
          <div class="stage-main">
            <div>
              <span class="stage-phase">${escapeHtml(stage.phase)}</span>
              <h3>${escapeHtml(stage.title)}</h3>
              <p class="stage-summary">${escapeHtml(stage.summary)}</p>
              <p class="stage-action"><strong>STEP / </strong>${escapeHtml(stage.action)}</p>
            </div>
            <button class="stage-toggle ${hasBugs ? "" : "stage-no-bugs"}"
              type="button" aria-expanded="false">
              OPEN STEP
            </button>
          </div>
          <div class="stage-details">
            ${implementationDetails(stage)}
            ${hasBugs ? `
              <section class="debug-section">
                <div class="debug-section-head">
                  <span>DEBUG HISTORY</span>
                  <p>${bugCount} observed problem${bugCount > 1 ? "s" : ""}, traced and resolved.</p>
                </div>
                <div class="bug-list">${stage.bugs.map(debugChain).join("")}</div>
              </section>` : result}
          </div>
        </div>
      </article>
    `;
  }).join("");

  document.querySelectorAll(".stage-toggle").forEach((button) => {
    button.addEventListener("click", () => {
      const stage = button.closest(".stage");
      const isOpen = stage.classList.toggle("open");
      button.setAttribute("aria-expanded", String(isOpen));
      button.textContent = isOpen ? "CLOSE STEP" : "OPEN STEP";
      if (isOpen) {
        history.replaceState(null, "", `#${stage.id}`);
      }
    });
  });

  const linkedStage = location.hash ? document.querySelector(location.hash) : null;
  if (linkedStage && linkedStage.classList.contains("stage")) {
    linkedStage.classList.add("open");
    const linkedButton = linkedStage.querySelector(".stage-toggle");
    linkedButton.setAttribute("aria-expanded", "true");
    linkedButton.textContent = "CLOSE STEP";
  }

  document.getElementById("restartFlow").innerHTML = data.restart
    .map((item, index) => `
      <article class="restart-state ${item.alert ? "alert" : ""} ${item.success ? "success" : ""}">
        <span class="restart-number">${String(index + 1).padStart(2, "0")}</span>
        <strong>${escapeHtml(item.state)}</strong>
        <p>${escapeHtml(item.detail)}</p>
      </article>
    `).join("");

  const matrixEntries = data.stages.flatMap((stage) =>
    (stage.bugs || []).map((bug) => ({
      symptom: bug.symptom,
      diagnosis: bug.diagnosis,
      fix: bug.fix
    }))
  );

  const renderMatrix = (query = "") => {
    const normalized = query.trim().toLowerCase();
    const visible = matrixEntries.filter((row) =>
      `${row.symptom} ${row.diagnosis} ${row.fix}`.toLowerCase().includes(normalized)
    );
    document.getElementById("matrixTable").innerHTML = `
      <div class="matrix-row header">
        <div>OBSERVED ERROR</div>
        <div>ROOT CAUSE</div>
        <div>RESOLUTION</div>
      </div>
      ${visible.map((row) => `
        <div class="matrix-row">
          <div>${escapeHtml(row.symptom)}</div>
          <div>${escapeHtml(row.diagnosis)}</div>
          <div>${escapeHtml(row.fix)}</div>
        </div>
      `).join("")}
    `;
  };
  renderMatrix();

  document.getElementById("matrixSearch").addEventListener("input", (event) => {
    renderMatrix(event.target.value);
  });

  document.querySelectorAll(".filter").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".filter").forEach((item) =>
        item.classList.toggle("active", item === button));
      const filter = button.dataset.filter;
      document.querySelectorAll(".stage").forEach((stage) => {
        const visible =
          filter === "all" ||
          (filter === "bugs" && stage.dataset.bugs !== "0") ||
          (filter === "critical" && stage.dataset.critical === "true");
        stage.classList.toggle("hidden", !visible);
      });
    });
  });

  document.querySelectorAll(".view-button").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".view-button").forEach((item) =>
        item.classList.toggle("active", item === button));
      document.body.dataset.view = button.dataset.view;
      window.scrollTo({ top: 0, behavior: "smooth" });
    });
  });

  const themeButton = document.getElementById("themeButton");
  themeButton.addEventListener("click", () => {
    const light = document.body.classList.toggle("light");
    themeButton.textContent = light ? "Dark" : "Light";
  });
})();
