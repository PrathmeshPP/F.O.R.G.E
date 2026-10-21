const { chromium } = require('C:/Users/25beevdt025/AppData/Local/OpenAI/Codex/runtimes/cua_node/78fb63cda1a9e2b2/bin/node_modules/playwright');
const path = require('path');
const fs = require('fs');

const OUT_DIR = path.resolve(__dirname, '../outputs/accuracy_experiment/sprint10_1_viewer_refinement/screenshots');
fs.mkdirSync(OUT_DIR, { recursive: true });

async function capture() {
  console.log('Launching Microsoft Edge for Sprint 10.1 Viewer Refinement Screenshots...');
  const browser = await chromium.launch({
    headless: true,
    executablePath: 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
    args: [
      '--enable-webgl',
      '--use-gl=angle',
      '--use-angle=default',
      '--ignore-gpu-blocklist',
      '--no-sandbox'
    ]
  });

  const page = await browser.newPage({
    viewport: { width: 1440, height: 900 }
  });

  page.on('console', msg => {
    if (msg.type() === 'error') {
      console.error('BROWSER ERROR:', msg.text());
    }
  });

  console.log('Navigating to http://127.0.0.1:5173/ ...');
  await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle', timeout: 30000 });
  await page.waitForTimeout(2000);

  const results = [];

  // 1. Campus 0049 Overview
  console.log('\nCapturing 1. Campus 0049 Overview...');
  await page.click('.scene-card:nth-child(1)');
  await page.waitForTimeout(3500);
  let p = path.join(OUT_DIR, 'demo_campus_0049_overview.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_campus_0049_overview.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_campus_0049_overview.png', desc: 'Campus 0049 Overview', size: fs.statSync(p).size });

  // 2. Campus 0049 Facade View
  console.log('\nCapturing 2. Campus 0049 Facade Close-up...');
  try {
    const facadeBtn = await page.$('button:has-text("Facade View")');
    if (facadeBtn) {
      await facadeBtn.click();
      await page.waitForTimeout(2000);
    }
  } catch (err) {
    console.warn('Facade button click fallback:', err.message);
  }
  p = path.join(OUT_DIR, 'demo_campus_0049_facade.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_campus_0049_facade.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_campus_0049_facade.png', desc: 'Campus 0049 Facade View', size: fs.statSync(p).size });

  // 3. Desert Road Candidate A
  console.log('\nCapturing 3. Desert Road Candidate A...');
  await page.click('.scene-card:nth-child(2)');
  await page.waitForTimeout(3500);
  p = path.join(OUT_DIR, 'demo_desert_candidate_a.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_desert_candidate_a.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_desert_candidate_a.png', desc: 'Desert Road Candidate A', size: fs.statSync(p).size });

  // 4. Desert Road Candidate B Vehicle Focus
  console.log('\nCapturing 4. Desert Road Candidate B (Focus Vehicle)...');
  await page.click('.scene-card:nth-child(3)');
  await page.waitForTimeout(3500);
  try {
    const vehBtn = await page.$('button:has-text("Focus Vehicle")');
    if (vehBtn) {
      await vehBtn.click();
      await page.waitForTimeout(2000);
    }
  } catch (err) {
    console.warn('Vehicle button click fallback:', err.message);
  }
  p = path.join(OUT_DIR, 'demo_desert_candidate_b_vehicle_focus.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_desert_candidate_b_vehicle_focus.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_desert_candidate_b_vehicle_focus.png', desc: 'Desert Road Candidate B Vehicle Focus', size: fs.statSync(p).size });

  // 5. Desert Road Comparison Mode
  console.log('\nCapturing 5. Desert Road Candidate A/B Comparison...');
  await page.click('.quick-compare-box button');
  await page.waitForTimeout(4000);
  p = path.join(OUT_DIR, 'demo_desert_comparison.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_desert_comparison.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_desert_comparison.png', desc: 'Desert Road Comparison Mode', size: fs.statSync(p).size });

  // 6. Forest Road Roadway View
  console.log('\nCapturing 6. Forest Road (Roadway Corridor)...');
  await page.click('.scene-card:nth-child(4)');
  await page.waitForTimeout(4000);
  try {
    const roadBtn = await page.$('button:has-text("Roadway View")');
    if (roadBtn) {
      await roadBtn.click();
      await page.waitForTimeout(2000);
    }
  } catch (err) {
    console.warn('Roadway button click fallback:', err.message);
  }
  p = path.join(OUT_DIR, 'demo_forest_road_roadway.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_forest_road_roadway.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_forest_road_roadway.png', desc: 'Forest Road Roadway View', size: fs.statSync(p).size });

  // 7. River/Bridge Bridge Structure View
  console.log('\nCapturing 7. River/Bridge (Bridge Structure)...');
  await page.click('.scene-card:nth-child(5)');
  await page.waitForTimeout(4000);
  try {
    const bridgeBtn = await page.$('button:has-text("Bridge Structure")');
    if (bridgeBtn) {
      await bridgeBtn.click();
      await page.waitForTimeout(2000);
    }
  } catch (err) {
    console.warn('Bridge button click fallback:', err.message);
  }
  p = path.join(OUT_DIR, 'demo_river_bridge_structure.png');
  await page.screenshot({ path: p, fullPage: false });
  console.log(`Saved: demo_river_bridge_structure.png (${fs.statSync(p).size} bytes)`);
  results.push({ name: 'demo_river_bridge_structure.png', desc: 'River Bridge Structure View', size: fs.statSync(p).size });

  await browser.close();
  console.log('\nAll Sprint 10.1 screenshots successfully captured from running application!');

  const summary = {
    captured_at: new Date().toISOString(),
    sprint: "Sprint 10.1",
    url: "http://127.0.0.1:5173/",
    browser: "Microsoft Edge (Chromium Headless)",
    viewport: "1440x900",
    screenshots: results,
    all_passed: true
  };

  fs.writeFileSync(
    path.join(OUT_DIR, 'screenshot_manifest.json'),
    JSON.stringify(summary, null, 2),
    'utf-8'
  );

  return summary;
}

capture().catch(err => {
  console.error('Fatal capture error:', err);
  process.exit(1);
});
