import torch
from peft import AutoPeftModelForCausalLM, peft_config
from transformers import AutoTokenizer, pipeline
from datasets import load_dataset


peft_model_id = "./opt-350m"
# peft_model_id = args.output_dir

# Load Model with PEFT adapter
model = AutoPeftModelForCausalLM.from_pretrained(
  peft_model_id,
  device_map="auto",
  torch_dtype=torch.float16
)

config = peft_config.AutoConfig.from_pretrained(peft_model_id)
# load into pipeline
pipe = pipeline("text-generation", model=model, tokenizer=AutoTokenizer.from_pretrained(config.tokenizer_name_or_path))

# Load our test dataset
eval_dataset = load_dataset("json", data_files="input.json", split="train")

# Test on sample
prompt = pipe.tokenizer.apply_chat_template(eval_dataset[0]["messages"][:2], tokenize=False, add_generation_prompt=True)
outputs = pipe(prompt, max_new_tokens=256, do_sample=False, temperature=0.1, top_k=50, top_p=0.1, eos_token_id=pipe.tokenizer.eos_token_id, pad_token_id=pipe.tokenizer.pad_token_id)

# print(f'prompt:\n {prompt}')
print(f"Query:\n{eval_dataset[0]['messages'][1]['content']}")
print(f"Original Answer:\n{eval_dataset[0]['messages'][2]['content']}")
print(f"Generated Answer:\n{outputs[0]['generated_text'][len(prompt):].strip()}")
