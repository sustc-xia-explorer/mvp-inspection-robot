#!/usr/bin/env python3
"""Freeze named planner profiles and inventory distinct experiment settings.
Original parameter files are retained. Engine identity is part of the key:
the same YAML can behave differently in the local and upstream cores.
"""
from pathlib import Path
import argparse,csv,hashlib,json,re
import yaml

PKG=Path(__file__).resolve().parents[1]
DEFAULT_WORK=PKG.parents[3]

def read(path):return yaml.safe_load(path.read_text())
def normalize(value):
 if isinstance(value,dict):return {str(k):normalize(v) for k,v in value.items()}
 if isinstance(value,list):return [normalize(v) for v in value]
 if isinstance(value,(int,float)) and not isinstance(value,bool):return float(value)
 return value
def digest(data):return hashlib.sha256(json.dumps(normalize(data),sort_keys=True,separators=(',',':')).encode()).hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('--work-root',type=Path,default=DEFAULT_WORK);p.add_argument('--new-config',type=Path,required=True);p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args()
 work=a.work_root.resolve();out=PKG/'config/profiles';out.mkdir(exist_ok=True)
 special=[
  ('FW-OMNI-FULL-V1','本地全向完整平滑版','ros1','ros1/fw_mini_omni_full_v1.yaml',out/'ros1/fw_mini_omni_full_v1.yaml' if (out/'ros1/fw_mini_omni_full_v1.yaml').exists() else PKG/'config/fw_mini_omni.yaml','validated_previous','历史完整平滑快照；T15/PAN4'),
  ('FW-OMNI-LIGHT-V1','本地全向小计算基准','ros1','ros1/fw_mini_omni_light_v1.yaml',work/'neupan_omni_diff_budget_validation_20261027/configs/small_budget.yaml','baseline','T10/PAN2；连续急弯低速 3.1 s'),
  ('FW-OMNI-LIGHT-TURN-V2','本地全向小计算急弯调优版','ros1','ros1/fw_mini_omni_light_turn_v2.yaml',a.new_config.resolve(),'validated_current','当前推荐默认；原 ROS1 核心，已完成工控机复测'),
  ('FW-OMNI-CORRIDOR-V1','本地全向旧走廊平滑版','ros1','ros1/fw_mini_omni_corridor_v1.yaml',PKG/'test/results/omni_smooth_20261027/ros1_periodic_sequence/world/planner.yaml','archive','加入完整转弯速度包络之前的版本'),
  ('FW-OMNI-LEGACY-V0','本地全向初版','ros1','ros1/fw_mini_omni_legacy_v0.yaml',PKG/'test/results/omni_smooth_20261027/baseline_planner.yaml','archive','早期 omni 参数，供回溯'),
  ('FW-DIFF-BALANCED-ARCHIVE','FW 差速均衡归档版','ros1','archive/fw_mini_diff_balanced_archive.yaml',PKG/'config/fw_mini_balanced.yaml','archive','旧 diff 基准；厂商已确认当前车不按锁后轮 diff 使用'),
  ('FW-DIFF-INITIAL-ARCHIVE','FW 差速初始归档版','ros1','archive/fw_mini_diff_initial_archive.yaml',PKG/'config/fw_mini.yaml','archive','旧 diff 初始参数'),
  ('FW-DIFF-SIM-ARCHIVE','FW 差速早期仿真版','ros1','archive/fw_mini_diff_sim_archive.yaml',PKG/'config/fw_mini_sim.yaml','archive','早期 RViz diff 配置'),
  ('UP-FW-OMNI-BASE-V1','上游全向 FW 初适配版','upstream','upstream/fw_mini_omni_base_v1.yaml',work/'neupan_upstream_omni_comparison_20261028/configs/upstream_fw_scaled.yaml','comparison','上游原核心，FW 车身/速度与 close_threshold 适配'),
  ('UP-FW-OMNI-TUNED-V2','上游全向 FW 调参版','upstream','upstream/fw_mini_omni_tuned_v2.yaml',work/'neupan_upstream_omni_tuning_20261028/fw_mini_upstream_tuned.yaml','validated_upstream','上游原核心专用；不能等价加载于当前 ROS1 核心'),
  ('UP-SENTRY-OMNI-REFERENCE','上游哨兵实车参考版','upstream','upstream/sentry_omni_reference.yaml',work/'neupan_cpp/neupan_cpp_ros/config/sentry_omni.yaml','other_robot','八边形哨兵、ref_speed 1.5 m/s，非 FW 模型'),
  ('EXP-REFERENCE-PORT','上游参考逻辑移植实验','reference_port','experiments/reference_port/fw_reference_port.yaml',work/'neupan_upstream_omni_comparison_20261028/configs/ported_reference_fw_scaled.yaml','experiment','仅配合隔离 candidate_core；并非当前正式 ROS1 核心'),
  ('EXP-OMNI-DIFF-COSTS','全向套用 diff 规划代价对照','ros1','experiments/ros1/fw_omni_diff_costs.yaml',work/'neupan_omni_diff_budget_validation_20261027/configs/diff_planner_omni_robot.yaml','comparison','omni 机器人段 + diff 规划代价，保留历史对照'),
  ('DIAG-OMNI-DIFF-UNITS','只改模型名的单位诊断','ros1','experiments/ros1/omni_diff_units_diagnostic.yaml',work/'neupan_omni_diff_budget_validation_20261027/configs/literal_diff_switch_only.yaml','diagnostic','方向角界限仍为 0.4 rad，不能当作通用 FW 全向配置'),
  ('EXAMPLE-GENERIC-DIFF','原仓库通用差速示例','ros1','examples/generic_diff.yaml',PKG/'config/planner.yaml','other_robot','1.6×2.0 m，非 FW-mini 几何'),
  ('SELFTEST-SMOKE','ROS1 冒烟自测配置','ros1','examples/smoke_selftest.yaml',PKG/'config/smoke.yaml','selftest','虚拟自测，非 FW 实车参数'),
 ]
 groups={};major=[]
 def add(source,engine,profile_id=None,label=None,destination=None,status='experiment',note=''):
  if not source.is_file():return
  try:data=read(source)
  except (OSError,yaml.YAMLError):return
  if not isinstance(data,dict) or not isinstance(data.get('robot'),dict) or not ('receding' in data or 'pan' in data):return
  sha=digest(data);key=(engine,sha)
  if key not in groups:
   if profile_id is None:profile_id=f'TRIAL-{engine.upper().replace("_","-")}-{sha[:10]}'
   if label is None:label='参数试验：'+source.stem+' / '+source.parent.name
   if destination is None:destination=f'experiments/{engine}/{profile_id.lower()}.yaml'
   file=out/destination;file.parent.mkdir(parents=True,exist_ok=True)
   header=f'# Profile: {profile_id}\n# Name: {label}\n# Engine: {engine}; status: {status}\n# Frozen copy. See config/profiles/README.md and index.yaml.\n'
   if file.exists() and digest(read(file))!=sha:
    raise SystemExit('Existing named profile differs; preserve it and assign a new version: '+str(file))
   if file.resolve()!=source.resolve():file.write_text(header+source.read_text())
   assert digest(read(file))==sha
   groups[key]=dict(id=profile_id,name=label,engine=engine,status=status,file=str(file.relative_to(PKG)),
      semantic_sha256=sha,notes=note,source_files=[],
      kinematics=data['robot'].get('kinematics','diff'),receding=data.get('receding'),pan_iterations=data.get('pan',{}).get('iter_num'),
      ref_speed=data.get('ref_speed'),control_change_weight=data.get('adjust',{}).get('control_change_weight'),
      max_acce=data['robot'].get('max_acce'))
  record=groups[key]
  try:alias=str(source.relative_to(work))
  except ValueError:alias=str(source)
  if alias not in record['source_files']:record['source_files'].append(alias)
  return record
 for ident,label,engine,dest,source,status,note in special:
  record=add(source,engine,ident,label,dest,status,note)
  if record and record['id'] not in major:major.append(record['id'])
 def engine(path,data):
  s=str(path)
  if '/developer_comparison/upstream_tuned/' in s or path.name=='upstream_fw_mini_upstream_tuned.yaml':return 'upstream'
  if 'ported_reference' in s or 'candidate_core' in s:return 'reference_port'
  if path.name in ['small_budget.yaml','light_v1.yaml']:return 'ros1'
  if 'neupan_local_light_turn_tuning_' in s:return 'ros1'
  if 'neupan_upstream_omni_tuning_' in s:return 'upstream'
  if 'neupan_upstream_omni_comparison_' in s:return 'upstream'
  if str(PKG) in s:return 'ros1'
  if str(work/'neupan_cpp') in s:return 'upstream'
  if 'neupan_runtime_comparison_' in s or 'neupan_omni_diff_budget_validation_' in s:return 'ros1'
  return 'archived_diff_core'
 roots=[PKG.parent,work/'neupan_cpp',work/'neupan_omni_diff_budget_validation_20261027/configs',
        work/'neupan_runtime_comparison_20261027/configs',work/'neupan_upstream_omni_comparison_20261028/configs',
        work/'neupan_upstream_omni_tuning_20261028/configs',a.report_dir/'configs']
 found=set()
 for root in roots:
  if not root.exists():continue
  for source in root.rglob('*.yaml'):
   if source in found or out in source.parents or any(x in source.parts for x in ['build','devel','install','.git','node_modules']):continue
   found.add(source)
   try:data=read(source)
   except (OSError,yaml.YAMLError):continue
   if isinstance(data,dict) and isinstance(data.get('robot'),dict) and ('receding' in data or 'pan' in data):add(source,engine(source,data))
 entries=sorted(groups.values(),key=lambda r:(r['id'] not in major,major.index(r['id']) if r['id'] in major else r['id']))
 assert len({r['id'] for r in entries})==len(entries)
 default_profile=groups.get(('ros1',digest(read(PKG/'config/fw_mini_omni.yaml'))),{}).get('id','CUSTOM')
 registry=dict(schema=1,description='Planner profiles, deduplicated by engine + normalized YAML. Maps/scenarios and duplicate export bundles are not planner profiles.',
   default_omni_entry='config/fw_mini_omni.yaml',default_omni_profile=default_profile,
   recommended_candidate='FW-OMNI-LIGHT-TURN-V2',engines={'ros1':'package core/; standard current neupan_cpp_ros1',
      'upstream':'zhangkaiyuan007/neupan_cpp 074718d4; supports projected_reference',
      'reference_port':'isolated experimental candidate_core; not standard ROS1',
      'archived_diff_core':'outer archived legacy C++ core'},profiles=entries)
 (out/'index.yaml').write_text(yaml.safe_dump(registry,sort_keys=False,allow_unicode=True))
 columns=['id','name','engine','status','file','kinematics','receding','pan_iterations','ref_speed','semantic_sha256','source_files']
 with (out/'all_profiles.csv').open('w',newline='') as stream:
  w=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore');w.writeheader()
  for r in entries:w.writerow({**r,'source_files':' | '.join(r['source_files'])})
 byid={r['id']:r for r in entries}
 lines=['# 规划参数配置命名与位置','',
 '本目录按运行核心区分参数，原文件保留。带版本号文件是冻结快照；原入口不会自动跟随快照变更。修改哪一份，运行时就通过 config_file 显式选择哪一份。','',
 '| 编号 | 中文名称 | 核心 | T / PAN | 文件 | 说明 |','|---|---|---|---|---|---|']
 for ident in major:
  r=byid[ident];file=str(Path(r['file']).relative_to('config/profiles'))
  lines.append(f"| {ident} | {r['name']} | {r['engine']} | {r['receding']} / {r['pan_iterations']} | [{Path(file).name}]({file}) | {r['notes']} |")
 lines+=['',f'当前 `config/fw_mini_omni.yaml` 对应 `{default_profile}`；`sim_complete.launch` 默认读取该入口。命名快照与入口不自动同步，运行其他快照时显式传入 config_file。','',
 '仅 `ros1/` 中的本地全向配置可直接配当前 ROS1 核心使用。`upstream/` 与 `reference_port` 记录用于对应的核心；当前正式 ROS1 核心不解析 projected_reference，不能把同一 YAML 在不同核心上当成等价试验。哨兵八边形和通用示例也不能使用 FW-mini 四边形模型。','',
 '本轮新配置仿真：','', '```bash',
 'roslaunch neupan_cpp_ros1 sim_complete.launch '+chr(92),
 '  config_file:="$(rospack find neupan_cpp_ros1)/config/profiles/ros1/fw_mini_omni_light_turn_v2.yaml"',
 '```','',
 f'共收录 {sum(len(r["source_files"]) for r in entries)} 个来源文件，按核心与参数内容去重为 {len(entries)} 套。完整编号和来源映射见 [index.yaml](index.yaml) 与 [all_profiles.csv](all_profiles.csv)。非主配置统一使用 TRIAL-核心-内容哈希编号；原试验名仍保留在 name/source_files 中，可搜索 rv16_rd0p5、q6_a012_w04 等旧名称。','',
 '扫描范围为项目主目录和本次性能/上游/急弯调参实验的配置；地图、场景 YAML、旧导出包副本单独保留，不重复作为规划器配置编号。','',
 '参数字段的中文名称与单位见 [CONFIG_PROFILES_CN.md](../../CONFIG_PROFILES_CN.md)。']
 (out/'README.md').write_text('\n'.join(lines)+'\n')
 print(json.dumps(dict(named_profiles=len(entries),source_files=sum(len(r['source_files']) for r in entries),major_profiles=major,registry=str(out/'index.yaml')),ensure_ascii=False))

if __name__=='__main__':main()
