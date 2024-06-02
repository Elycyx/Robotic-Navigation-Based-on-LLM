#!/usr/bin/env python
# coding: utf-8

import os
from datetime import datetime
import torch
from datasets import load_dataset
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig, TrainingArguments, pipeline
from trl import setup_chat_format, SFTTrainer
from peft import LoraConfig, AutoPeftModelForCausalLM
from tqdm import tqdm
import json
from peft import AutoPeftModelForCausalLM

# Set Hugging Face mirror and CUDA configuration
os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'expandable_segments:True'

# Load datasets
dataset = load_dataset("json", data_files="hospital_dataset.json", split="train")
validation_dataset = load_dataset("json", data_files="hospital_test.json", split="train")
eval_dataset = validation_dataset

# Hugging Face model ID
model_id = "openlm-research/open_llama_3b" 

def evaluate_position(sample):
    try:
        prompt = pipe.tokenizer.apply_chat_template(sample["messages"][:2], tokenize=False, add_generation_prompt=True)
        outputs = pipe(prompt, max_new_tokens=256, do_sample=True, temperature=0.7, top_k=50, top_p=0.95, eos_token_id=pipe.tokenizer.eos_token_id, pad_token_id=pipe.tokenizer.pad_token_id)
        # Extract position information from the validation set
        generated_positions = json.loads(sample["messages"][2]["content"])["positions"]
        # Extract generated position information
        true_positions = json.loads(outputs[0]["generated_text"][len(prompt):].strip())["positions"]
        # Compare the generated position information with the validation set position information
        if generated_positions == true_positions:
            return 1
        else:
            return 0
    except json.JSONDecodeError:
        # If unable to parse as JSON, output "error format" directly
        return 0

class AccuracyCallback(TrainerCallback):
    """Custom callback to evaluate model accuracy at the end of each training epoch."""
    def on_epoch_end(self, args, state: TrainerState, control: TrainerControl, **kwargs):
        # Use your existing evaluation logic here
        success_rate = []
        eval_dataset = load_dataset("json", data_files="hospital_test.json", split="train")
        for sample in eval_dataset:
            success_rate.append(evaluate_position(sample))
        accuracy = sum(success_rate) / len(success_rate)
        log_file.write(f"Accuracy after epoch {state.epoch}: {accuracy * 100:.2f}%")

# BitsAndBytesConfig configuration
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True, bnb_4bit_use_double_quant=True, bnb_4bit_quant_type="nf4", bnb_4bit_compute_dtype=torch.bfloat16
)

# Load model and tokenizer
model = AutoModelForCausalLM.from_pretrained(
    model_id,
    device_map="auto",
    torch_dtype=torch.bfloat16,
    quantization_config=bnb_config
)
tokenizer = AutoTokenizer.from_pretrained(model_id)
tokenizer.padding_side = 'right'

# Set chat template
model, tokenizer = setup_chat_format(model, tokenizer)

# LoRA configuration
peft_config = LoraConfig(
        lora_alpha=128,
        lora_dropout=0.05,
        r=256,
        bias="none",
        target_modules=["q_proj", "v_proj"],
        task_type="CAUSAL_LM",
)

# Training parameters
initial_learning_rate = 0.0001
learning_rate_step = 0.0001

learning_rate = initial_learning_rate

while learning_rate <= 0.001:  # Until the maximum learning rate is reached
    num_epochs = 25
    log_file = open(f"fine_tuning_logs_loss{learning_rate}.txt", "w")
    args = TrainingArguments(
        output_dir=f"{learning_rate}model",  # Directory to save the model
        num_train_epochs=num_epochs,  # Number of training epochs
        per_device_train_batch_size=2,  # Batch size per device
        gradient_accumulation_steps=2,  # Number of steps before performing a backward/update pass
        gradient_checkpointing=True,  # Use gradient checkpointing to save memory
        optim="adamw_torch_fused",  # Use fused AdamW optimizer
        logging_steps=10,  # Log every 10 steps
        save_strategy="epoch",  # Save checkpoint every epoch
        learning_rate=learning_rate,  # Learning rate
        bf16=True,  # Use bfloat16 precision
        tf32=True,  # Use tf32 precision
        max_grad_norm=0.3,  # Maximum gradient norm
        warmup_ratio=0.03,  # Warmup ratio
        lr_scheduler_type="constant",  # Use constant learning rate scheduler
        push_to_hub=True,  # Do not push the model to hub
        report_to="tensorboard",  # Report metrics to tensorboard
        eval_steps=10,  # Evaluate every 10 steps
        evaluation_strategy="no",  # Evaluate every epoch
    )
    max_seq_length = 3072

    # Create new trainer instance
    trainer = SFTTrainer(
        model=model,
        args=args,
        train_dataset=dataset,
        eval_dataset=validation_dataset,
        peft_config=peft_config,
        max_seq_length=max_seq_length,
        tokenizer=tokenizer,
        packing=True,
        dataset_kwargs={
            "add_special_tokens": False,  # Use special tokens
            "append_concat_token": False,  # No need to add extra separator tokens
        },
        callbacks=[AccuracyCallback()]
    )

    # Train the model
    trainer.train()

    # Save the model
    trainer.save_model()
    
    # Load PEFT model on CPU
    model = AutoPeftModelForCausalLM.from_pretrained(
        args.output_dir,
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True,
    )
    # Merge LoRA and base model and save
    merged_model = model.merge_and_unload()
    merged_model.save_pretrained(args.output_dir, safe_serialization=True, max_shard_size="2GB")

    # Free the memory again
    del model
    del trainer
    torch.cuda.empty_cache()

    # Record the fine-tuning process logs into a txt file
    log_file.write(f"Fine-tuning logs for learning rate: {learning_rate}")
    for log in trainer.state.log_history:
        log_file.write(str(log) + "\n")

    # Evaluate the model
    eval_dataset = load_dataset("json", data_files="hospital_test.json", split="train")
    peft_model_id = f"./{learning_rate}model"
    test_model = AutoPeftModelForCausalLM.from_pretrained(
        peft_model_id,
        device_map="auto",
        torch_dtype=torch.float16
    )
    tokenizer = AutoTokenizer.from_pretrained(peft_model_id)
    pipe = pipeline("text-generation", model=test_model, tokenizer=tokenizer)

    # Increase the learning rate
    learning_rate += learning_rate_step
