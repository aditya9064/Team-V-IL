"""IndustReal-derived FANUC insertion research. Importing does not launch Isaac."""

TASK_ID = "TeamV-Fanuc-PegInsert-Direct-v0"


def register():
    import gymnasium as gym
    if TASK_ID not in gym.registry:
        gym.register(
            id=TASK_ID,
            entry_point="team_v_industreal.environment:FanucInsertionEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": "team_v_industreal.configuration:FanucPegInsertCfg",
                "rl_games_cfg_entry_point": "team_v_industreal:agents/ppo.yaml",
            },
        )
